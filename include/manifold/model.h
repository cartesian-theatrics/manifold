// Copyright 2026. Licensed under the Apache License, Version 2.0.
#pragma once

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#include "manifold/manifold.h"

namespace manifold {

// Native value types: neither a language-side registry nor inheritance from
// Manifold. Its nonvirtual, Manifold-returning methods would slice appearance.
struct ModelImage {
  const int width, height;
  const std::vector<uint8_t> rgba;  // sRGB RGB, linear alpha, top row first
  ModelImage(int w, int h, std::vector<uint8_t> pixels)
      : width(w), height(h), rgba(std::move(pixels)) {
    if (w < 1 || h < 1 || uint64_t(w) * h > 16777216 ||
        rgba.size() != uint64_t(w) * h * 4)
      throw std::invalid_argument("Image requires 1..16777216 RGBA pixels");
  }
};

struct ModelLayer {
  uint64_t id;
  std::string name;
  std::shared_ptr<const ModelImage> image;
  int uvIndex;  // absolute MeshGL channel, including XYZ
  double opacity;
  bool repeat;
  std::array<double, 4> coverage;
};

struct ModelAppearance {
  vec4 color = {1, 1, 1, 1};
  int colorIndex = -1;
  std::vector<ModelLayer> layers;  // straight-alpha source-over, oldest first
};

struct ModelBake {
  MeshGL mesh;  // XYZ, UV, NORMAL; geometric seams retained
  int width = 0, height = 0;
  std::vector<uint8_t> rgba;
};

class Model {
 public:
  Model() = default;
  explicit Model(const Manifold& geometry) : geometry_(geometry) {
    CheckGeometry(geometry);
    ModelAppearance appearance;
    if (geometry.NumProp() == 4) appearance.colorIndex = 3;
    for (uint32_t id : geometry.GetMeshGL64().runOriginalID)
      surfaces_.emplace(id, appearance);
    Prune();
  }
  // Explicit escape hatch. Calling this deliberately discards appearance.
  Manifold Geometry() const { return geometry_; }
  MeshGL GetMeshGL(int normalIdx = -1) const { return geometry_.GetMeshGL(normalIdx); }
  MeshGL64 GetMeshGL64(int normalIdx = -1) const { return geometry_.GetMeshGL64(normalIdx); }
  const std::map<uint32_t, ModelAppearance>& Surfaces() const { return surfaces_; }
  size_t LayerCount() const {
    std::set<uint64_t> ids;
    for (const auto& s : surfaces_) for (const auto& l : s.second.layers) ids.insert(l.id);
    return ids.size();
  }
  size_t SurfaceCount() const { return surfaces_.size(); }
  size_t ImageCount() const {
    std::set<const ModelImage*> images;
    for (const auto& s : surfaces_) for (const auto& l : s.second.layers) images.insert(l.image.get());
    return images.size();
  }
  int NextProperty() const { return int(geometry_.NumProp()) + 3; }

  // Mapping is executed once in native code, not a callback per vertex. The
  // mapper appends UV channels and preserves original-ID provenance.
  using Mapping = std::function<Manifold(const Manifold&, int)>;
  Model Texture(std::shared_ptr<const ModelImage> image, const Mapping& mapping,
                const std::string& name = "", double opacity = 1,
                bool repeat = false, std::array<double, 4> coverage = {0,0,1,1}) const {
    if (!image || !mapping || !std::isfinite(opacity) || opacity < 0 || opacity > 1)
      throw std::invalid_argument("Texture requires an image, mapping and opacity in [0,1]");
    for (double x : coverage) if (!std::isfinite(x)) throw std::invalid_argument("Invalid UV coverage");
    if (coverage[0] >= coverage[2] || coverage[1] >= coverage[3]) throw std::invalid_argument("Invalid UV coverage");
    int index = NextProperty();
    Model result = Keep(mapping(geometry_, index));
    if (result.NextProperty() < index + 2) throw std::invalid_argument("Mapping did not append UV channels");
    ModelLayer layer{NextLayerID(), name, std::move(image), index, opacity, repeat, coverage};
    for (auto& surface : result.surfaces_) surface.second.layers.push_back(layer);
    return result.Reidentify();
  }

  Model Color(vec4 rgba) const {
    for (int i = 0; i < 4; ++i) if (!std::isfinite(rgba[i]) || rgba[i] < 0 || rgba[i] > 1)
      throw std::invalid_argument("Color must be finite linear RGBA in [0,1]");
    Model result = *this;
    for (auto& surface : result.surfaces_) {
      surface.second.color = rgba;
      surface.second.colorIndex = -1;
    }
    return result.Reidentify();
  }

  Model Boolean(const Model& other, OpType op) const {
    // Different copies may share original IDs. Rebase the RHS at this native
    // boundary so neither branch's appearance can overwrite the other.
    Model right = other.Reidentify();
    Model result;
    result.geometry_ = geometry_.Boolean(right.geometry_, op);
    CheckGeometry(result.geometry_);
    result.surfaces_ = surfaces_;
    result.surfaces_.insert(right.surfaces_.begin(), right.surfaces_.end());
    result.Prune();
    return result;
  }
  Model operator+(const Model& b) const { return Boolean(b, OpType::Add); }
  Model operator-(const Model& b) const { return Boolean(b, OpType::Subtract); }
  Model operator^(const Model& b) const { return Boolean(b, OpType::Intersect); }
  Model Compose(const Model& other) const {
    Model right = other.Reidentify();
    Model result;
    result.geometry_ = Manifold::Compose({geometry_, right.geometry_});
    CheckGeometry(result.geometry_);
    result.surfaces_ = surfaces_;
    result.surfaces_.insert(right.surfaces_.begin(), right.surfaces_.end());
    result.Prune();
    return result;
  }
  Model Translate(vec3 v) const { return Keep(geometry_.Translate(v)); }
  Model Scale(vec3 v) const { return Keep(geometry_.Scale(v)); }
  Model Rotate(double x, double y = 0, double z = 0) const { return Keep(geometry_.Rotate(x,y,z)); }
  Model Mirror(vec3 v) const { return Keep(geometry_.Mirror(v)); }
  Model Transform(const mat3x4& m) const { return Keep(geometry_.Transform(m)); }
  Model Warp(std::function<void(vec3&)> f) const { return Keep(geometry_.Warp(f)); }
  Model Refine(int n) const { return Keep(geometry_.Refine(n)); }
  Model RefineToLength(double n) const { return Keep(geometry_.RefineToLength(n)); }
  Model RefineToTolerance(double n) const { return Keep(geometry_.RefineToTolerance(n)); }
  Model SmoothOut(double angle = 60, double smoothness = 0) const { return Keep(geometry_.SmoothOut(angle,smoothness)); }
  Model SetTolerance(double n) const { return Keep(geometry_.SetTolerance(n)); }
  Model CalculateNormals(int index, double angle = 60) const {
    if (index + 3 < NextProperty()) throw std::invalid_argument("Normals must append after model properties");
    return Keep(geometry_.CalculateNormals(index,angle));
  }
  std::vector<Model> Decompose() const {
    std::vector<Model> result;
    for (const auto& part : geometry_.Decompose()) result.push_back(Keep(part));
    return result;
  }
  std::pair<Model,Model> Split(const Model& cutter) const { return {*this ^ cutter, *this - cutter}; }
  // New hull/cap surfaces have no inherited parameterization. They must not be
  // silently assigned somebody else's image; those APIs are deliberately not
  // forwarded until they accept an explicit new-surface material policy.
  Manifold::Error Status() const { return geometry_.Status(); }
  bool IsEmpty() const { return geometry_.IsEmpty(); }
  size_t NumVert() const { return geometry_.NumVert(); }
  size_t NumTri() const { return geometry_.NumTri(); }
  size_t NumEdge() const { return geometry_.NumEdge(); }
  size_t NumProp() const { return geometry_.NumProp(); }
  size_t NumPropVert() const { return geometry_.NumPropVert(); }
  Box BoundingBox() const { return geometry_.BoundingBox(); }
  int Genus() const { return geometry_.Genus(); }
  double Volume() const { return geometry_.Volume(); }
  double SurfaceArea() const { return geometry_.SurfaceArea(); }
  double MinGap(const Model& b, double length) const { return geometry_.MinGap(b.geometry_, length); }
  Polygons Slice(double z = 0) const { return geometry_.Slice(z); }
  Polygons Project() const { return geometry_.Project(); }

  vec4 SampleColor(size_t triangle, double b1, double b2) const {
    if (!std::isfinite(b1) || !std::isfinite(b2) || b1 < 0 || b2 < 0 || b1+b2 > 1)
      throw std::invalid_argument("Invalid barycentric coordinates");
    auto mesh = GetMeshGL64();
    if (triangle >= mesh.NumTri()) throw std::out_of_range("Triangle index");
    return Sample(mesh, Appearance(mesh, triangle), triangle, b1, b2);
  }

  // Portable GLB has no general ordered-layer shader. Bake only the render
  // representation; retain original images, UVs and editable layers in Model.
  // Each triangle receives a guttered atlas tile; tileSize controls sampling
  // quality, independently of the native geometry's tessellation.
  ModelBake Bake(int tileSize = 16) const {
    if (tileSize < 2 || tileSize > 256) throw std::invalid_argument("Atlas tile size must be 2..256");
    if (IsEmpty()) throw std::invalid_argument("Cannot render an empty model");
    const int normalIndex = int(NumProp());
    auto input = geometry_.CalculateNormals(normalIndex,55).GetMeshGL64();
    const size_t count = input.NumTri();
    const int gutter = 4;
    int columns = int(std::ceil(std::sqrt(double(count)))), stride = tileSize + 2*gutter;
    int rows = int((count + columns - 1) / columns);
    if (uint64_t(columns) * rows * stride * stride > 67108864)
      throw std::invalid_argument("Atlas exceeds 64 million pixels; reduce :tile-size or model complexity");
    ModelBake result;
    result.width = columns * stride; result.height = rows * stride;
    result.rgba.resize(size_t(result.width) * result.height * 4);
    // Unused cells cannot make an otherwise opaque material transparent.
    for (size_t i=3; i<result.rgba.size(); i+=4) result.rgba[i]=255;
    result.mesh.numProp = 8;
    result.mesh.tolerance = input.tolerance;
    std::vector<uint32_t> first(input.NumVert(), UINT32_MAX);
    // Track physical vertices, including discontinuous original UV/color rows.
    std::vector<uint64_t> parent(input.NumVert());
    for (size_t i=0;i<parent.size();++i) parent[i]=i;
    auto root=[&](uint64_t i) { while(parent[i]!=i) i=parent[i]; return i; };
    for(size_t i=0;i<input.mergeFromVert.size();++i) parent[root(input.mergeFromVert[i])]=root(input.mergeToVert[i]);
    auto position=[&](size_t face,int corner) {
      const double* p=&input.vertProperties[input.triVerts[3*face+corner]*input.numProp];
      return vec3{p[0],p[1],p[2]};
    };
    std::vector<double> lengths(count);
    for(size_t f=0;f<count;++f) for(int c=0;c<3;++c)
      lengths[f]=std::max(lengths[f],la::length(position(f,c)-position(f,(c+1)%3)));
    std::nth_element(lengths.begin(),lengths.begin()+count/2,lengths.end());
    const double typicalLength=lengths[count/2];
    for (size_t face=0; face<count; ++face) {
      const auto& appearance = Appearance(input, face);
      int tx = int(face % columns) * stride, ty = int(face / columns) * stride;
      // A chart congruent to the physical triangle avoids huge derivatives on
      // slivers. Full-size right-triangle tiles cause MSAA fragment centers to
      // extrapolate into unrelated tiles, even when the covered samples lie
      // inside the triangle. Also bound density for subpixel-sized triangles.
      int a=0;
      for(int c=1;c<3;++c)
        if(la::length(position(face,c)-position(face,(c+1)%3))>
           la::length(position(face,a)-position(face,(a+1)%3))) a=c;
      int b=(a+1)%3,c=(a+2)%3;
      vec3 edge=position(face,b)-position(face,a), other=position(face,c)-position(face,a);
      double length=la::length(edge), density=(tileSize-1)/std::max(length,typicalLength);
      double x=la::dot(other,edge)/length;
      double height=la::length(la::cross(edge,other))/length;
      std::array<vec2,3> chart;
      chart[a]={0,0}; chart[b]={length*density,0}; chart[c]={x*density,height*density};
      auto barycentric=[&](vec2 point) {
        double v=point.y/chart[c].y, u=(point.x-v*chart[c].x)/chart[b].x;
        std::array<double,3> weights{};
        if(u>=0 && v>=0 && u+v<=1) { weights[a]=1-u-v; weights[b]=u; weights[c]=v; }
        else {
          double distance=INFINITY;
          // Extend the nearest edge/corner color throughout each tile gutter.
          for(int i=0;i<3;++i) {
            int j=(i+1)%3; vec2 d=chart[j]-chart[i]; double squared=la::dot(d,d);
            double t=squared==0 ? 0 : std::clamp(la::dot(point-chart[i],d)/squared,0.0,1.0);
            vec2 delta=point-(chart[i]+t*d); double candidate=la::dot(delta,delta);
            if(candidate<distance) {distance=candidate;weights={0,0,0};weights[i]=1-t;weights[j]=t;}
          }
        }
        return vec2{weights[1],weights[2]};
      };
      vec4 previous={-1,-1,-1,-1};
      std::array<uint8_t,4> encoded{};
      for (int y=0;y<stride;++y) for (int x=0;x<stride;++x) {
        vec2 uv=barycentric({double(x-gutter),double(y-gutter)});
        vec4 color=Sample(input,appearance,face,uv.x,uv.y);
        size_t offset=4*(size_t(ty+y)*result.width+tx+x);
        if(color.x!=previous.x || color.y!=previous.y || color.z!=previous.z || color.w!=previous.w) {
          for(int c=0;c<3;++c) encoded[c]=Encode(color[c]);
          encoded[3]=uint8_t(std::lround(std::clamp(color.w,0.0,1.0)*255));
          previous=color;
        }
        std::copy(encoded.begin(),encoded.end(),result.rgba.begin()+offset);
      }
      for(int corner=0;corner<3;++corner) {
        uint64_t vertex=input.triVerts[3*face+corner], physical=root(vertex);
        uint32_t index=uint32_t(result.mesh.NumVert());
        const double* p=&input.vertProperties[vertex*input.numProp];
        result.mesh.vertProperties.insert(result.mesh.vertProperties.end(),
          {float(p[0]),float(p[1]),float(p[2]),
           float((tx+gutter+0.5+chart[corner].x)/result.width),
           float((ty+gutter+0.5+chart[corner].y)/result.height),
           float(p[normalIndex+3]),float(p[normalIndex+4]),float(p[normalIndex+5])});
        result.mesh.triVerts.push_back(index);
        if(first[physical]==UINT32_MAX) first[physical]=index;
        else { result.mesh.mergeFromVert.push_back(index); result.mesh.mergeToVert.push_back(first[physical]); }
      }
    }
    result.mesh.runIndex.assign(input.runIndex.begin(),input.runIndex.end());
    result.mesh.runOriginalID=input.runOriginalID;
    result.mesh.runTransform.assign(input.runTransform.begin(),input.runTransform.end());
    result.mesh.faceID.assign(input.faceID.begin(),input.faceID.end());
    return result;
  }

 private:
  Manifold geometry_;
  std::map<uint32_t,ModelAppearance> surfaces_;
  static uint64_t NextLayerID() { static std::atomic<uint64_t> next{1}; return next.fetch_add(1); }
  static void CheckGeometry(const Manifold& geometry) {
    if(geometry.Status()!=Manifold::Error::NoError) throw std::invalid_argument("Operation produced invalid Model geometry");
  }
  void Prune() {
    std::set<uint32_t> live;
    const auto mesh=geometry_.GetMeshGL64();
    for(size_t run=0;run<mesh.runOriginalID.size();++run) {
      // Decompose and booleans may retain zero-length source runs. They own
      // no faces and must not keep an unrelated image alive in the result.
      if(mesh.runIndex[run]==mesh.runIndex[run+1]) continue;
      uint32_t id=mesh.runOriginalID[run];
      if(!surfaces_.count(id)) throw std::invalid_argument("Operation lost surface appearance provenance");
      live.insert(id);
    }
    for(auto it=surfaces_.begin();it!=surfaces_.end();) if(!live.count(it->first)) it=surfaces_.erase(it); else ++it;
  }
  Model Keep(const Manifold& geometry) const {
    CheckGeometry(geometry);
    Model result=*this; result.geometry_=geometry; result.Prune(); return result;
  }
  Model Reidentify() const {
    if(IsEmpty()) return *this;
    auto mesh=GetMeshGL64();
    Model result;
    std::map<uint32_t,uint32_t> ids;
    for(auto& id:mesh.runOriginalID) {
      auto found=ids.find(id);
      if(found==ids.end()) {
        uint32_t replacement=Manifold::ReserveIDs(1);
        if(surfaces_.count(id)) result.surfaces_.emplace(replacement,surfaces_.at(id));
        found=ids.emplace(id,replacement).first;
      }
      id=found->second;
    }
    result.geometry_=Manifold(mesh); CheckGeometry(result.geometry_); return result;
  }
  const ModelAppearance& Appearance(const MeshGL64& mesh,size_t triangle) const {
    auto it=std::upper_bound(mesh.runIndex.begin(),mesh.runIndex.end(),uint64_t(3*triangle));
    size_t run=size_t(it-mesh.runIndex.begin())-1;
    return surfaces_.at(mesh.runOriginalID.at(run));
  }
  static double Decode(uint8_t c) {
    static const std::array<double,256> linear=[] {
      std::array<double,256> table{};
      for(size_t i=0;i<table.size();++i) {
        double x=i/255.0;
        table[i]=x<=0.04045 ? x/12.92 : std::pow((x+0.055)/1.055,2.4);
      }
      return table;
    }();
    return linear[c];
  }
  static uint8_t Encode(double x) {
    x=std::clamp(x,0.0,1.0);
    return uint8_t(std::lround(255*(x<=0.0031308 ? 12.92*x : 1.055*std::pow(x,1/2.4)-0.055)));
  }
  static vec4 Sample(const MeshGL64& mesh,const ModelAppearance& appearance,size_t face,double u,double v) {
    const double *a=&mesh.vertProperties[mesh.triVerts[face*3]*mesh.numProp],
                 *b=&mesh.vertProperties[mesh.triVerts[face*3+1]*mesh.numProp],
                 *c=&mesh.vertProperties[mesh.triVerts[face*3+2]*mesh.numProp];
    auto property=[&](int i) { return (1-u-v)*a[i]+u*b[i]+v*c[i]; };
    vec4 result=appearance.color;
    if(appearance.colorIndex>=0) for(int i=0;i<4;++i) result[i]*=property(appearance.colorIndex+i);
    for(const auto& layer:appearance.layers) {
      double x=property(layer.uvIndex),y=property(layer.uvIndex+1);
      if(layer.repeat) { x-=std::floor(x); y-=std::floor(y); }
      else if(x<layer.coverage[0]-1e-9 || x>layer.coverage[2]+1e-9 || y<layer.coverage[1]-1e-9 || y>layer.coverage[3]+1e-9) continue;
      const auto& image=*layer.image;
      x=x*image.width-0.5; y=y*image.height-0.5;
      if(!layer.repeat) {
        x=std::clamp(x,0.0,double(image.width-1));
        y=std::clamp(y,0.0,double(image.height-1));
      }
      int ix=int(std::floor(x)),iy=int(std::floor(y)); double fx=x-ix,fy=y-iy;
      vec4 foreground={0,0,0,0};
      for(int dy=0;dy<2;++dy) for(int dx=0;dx<2;++dx) {
        int px=layer.repeat ? (ix+dx+image.width)%image.width : std::min(ix+dx,image.width-1);
        int py=layer.repeat ? (iy+dy+image.height)%image.height : std::min(iy+dy,image.height-1);
        size_t offset=4*(size_t(py)*image.width+px);
        double weight=(dx?fx:1-fx)*(dy?fy:1-fy), alpha=image.rgba[offset+3]/255.0;
        for(int i=0;i<3;++i) foreground[i]+=weight*alpha*Decode(image.rgba[offset+i]);
        foreground.w+=weight*alpha;
      }
      double alpha=foreground.w*layer.opacity, combined=alpha+result.w*(1-alpha);
      for(int i=0;i<3;++i) result[i]=combined==0 ? 0 : (foreground[i]*layer.opacity+result[i]*result.w*(1-alpha))/combined;
      result.w=combined;
    }
    return result;
  }
};
}  // namespace manifold
