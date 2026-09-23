#pragma once
#include "mesh_utils.hpp"
#include "manifold/model.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"
#include <cstring>
#include <sstream>
#include <unordered_map>

namespace manifold {
// This bridge lives beside MeshUtils because its native surface mapper and
// image codecs are shared by Java and WASM. No language-owned appearance state.
struct ModelTexture {
  std::string mapping="geodesic", name;
  vec3 origin={0,0,0}, normal={0,0,0}, right={1,0,0};
  double width=1, height=1, pixelSize=0;
  double u0=0,v0=0,u1=1,v1=1,opacity=1;
  int axisU=0,axisV=2;
  double scaleU=1,scaleV=1,offsetU=0,offsetV=0;
  double seamAngle=45,padding=0.01;
  bool pack=true;
  int depthWidth=0,depthHeight=0;
  double depthScale=1,depthOffset=0,depthFade=-1;
  bool step=false;
  std::vector<double> depth;
  void SetDepth(const double* values,size_t count,int w,int h) {
    if(w<2 || h<2 || uint64_t(w)*h!=count || count>2000000) throw std::invalid_argument("Invalid depth grid");
    depth.assign(values,values+count); depthWidth=w; depthHeight=h;
  }
  void SetDepthImage(const uint8_t* bytes,size_t count) {
    int w=0,h=0,n=0;
    if(!bytes || count<8 || count>67108864 || !stbi_info_from_memory(bytes,int(count),&w,&h,&n)
       || w<2 || h<2 || uint64_t(w)*h>2000000) throw std::invalid_argument("Invalid depth image");
    std::unique_ptr<stbi_us,decltype(&stbi_image_free)> pixels(stbi_load_16_from_memory(bytes,int(count),&w,&h,&n,1),stbi_image_free);
    if(!pixels) throw std::invalid_argument("Cannot decode depth image");
    depth.resize(size_t(w)*h); depthWidth=w; depthHeight=h;
    for(size_t i=0;i<depth.size();++i) depth[i]=pixels.get()[i]/65535.0;
  }
};

class ModelIO {
 public:
  static std::shared_ptr<const ModelImage> Decode(const uint8_t* bytes,size_t count) {
    if(!bytes || count>67108864 || count<8) throw std::invalid_argument("Expected PNG/JPEG image bytes (max 64 MiB)");
    int w=0,h=0,n=0;
    if(!stbi_info_from_memory(bytes,int(count),&w,&h,&n) || w<1 || h<1 || uint64_t(w)*h>16777216)
      throw std::invalid_argument("Cannot decode texture image, or image exceeds 16 million pixels");
    std::unique_ptr<uint8_t,decltype(&stbi_image_free)> pixels(stbi_load_from_memory(bytes,int(count),&w,&h,&n,4),stbi_image_free);
    if(!pixels) throw std::invalid_argument("Cannot decode texture image");
    return std::make_shared<const ModelImage>(w,h,std::vector<uint8_t>(pixels.get(),pixels.get()+size_t(w)*h*4));
  }
  static Model Texture(const Model& model,const uint8_t* bytes,size_t count,const ModelTexture& p) {
    auto image=Decode(bytes,count);
    if(p.mapping!="planar" && p.mapping!="geodesic" && p.mapping!="box" && p.mapping!="unwrap")
      throw std::invalid_argument("Mapping must be :planar, :geodesic, :box or :unwrap");
    if(p.mapping!="geodesic" && !p.depth.empty()) throw std::invalid_argument("Depth requires geodesic mapping");
    if(p.mapping=="box") {
      return model.Texture(image,[&](const Manifold& g,int index) {
        return MeshUtils::ApplyBoxUV(g,index,p.origin.x,p.origin.y,p.origin.z,p.width,p.height,
                                     p.scaleU,p.scaleV,p.offsetU,p.offsetV);
      },p.name,p.opacity,true);
    }
    if(p.mapping=="unwrap") {
      if(p.scaleU!=p.scaleV) throw std::invalid_argument("Unwrap requires a uniform scale");
      return model.Texture(image,[&](const Manifold& g,int index) {
        return MeshUtils::UnwrapUV(g,index,p.seamAngle,p.scaleU,p.padding,p.pack);
      },p.name,p.opacity,!p.pack);
    }
    if(p.mapping=="planar") {
      if(!p.depth.empty()) throw std::invalid_argument("Depth requires geodesic mapping");
      return model.Texture(image,[&](const Manifold& g,int index) {
        return MeshUtils::ApplyPlanarUV(g,index,p.axisU,p.axisV,p.scaleU,p.scaleV,p.offsetU,p.offsetV);
      },p.name,p.opacity,true);
    }
    double pixel=p.pixelSize==0 ? std::min(p.width,p.height)/32 : p.pixelSize;
    double fade=p.depthFade<0 ? (p.step ? 0 : std::min(2*pixel,std::min(p.width,p.height)/2)) : p.depthFade;
    MeshUtils::SurfaceUV::DepthField depth;
    if(!p.depth.empty()) depth={p.depth,p.depthWidth,p.depthHeight,p.depthScale,p.depthOffset,fade,p.step};
    return model.Texture(image,[&](const Manifold& g,int index) {
      return MeshUtils::GeodesicUV(g,index,p.origin.x,p.origin.y,p.origin.z,
        p.normal.x,p.normal.y,p.normal.z,p.right.x,p.right.y,p.right.z,
        p.width,p.height,p.u0,p.v0,p.u1-p.u0,p.v1-p.v0,-1,-1,pixel,depth);
    },p.name,p.opacity,false,{p.u0,p.v0,p.u1,p.v1});
  }

  // Preserve source images and UVs whenever standard glTF can represent the
  // appearance directly. The fallback compositor handles alpha layering and
  // triangles crossing a coverage boundary; no approximation drops a layer.
  static std::vector<uint8_t> ExportGLB(const Model& model,int tileSize=16) {
    if(tileSize<2 || tileSize>256) throw std::invalid_argument("Atlas tile size must be 2..256");
    if(model.IsEmpty()) throw std::invalid_argument("Cannot render an empty model");
    RenderData data;
    if(!Direct(model,data)) data=Baked(model,tileSize);
    return EncodeGLB(data);
  }

 private:
  struct Primitive {
    std::vector<float> positions, uv, normals, colors;
    std::vector<uint32_t> indices;
    vec4 color={1,1,1,1};
    int image=-1;
    bool repeat=false, transparent=false;
    vec3 low={INFINITY,INFINITY,INFINITY}, high={-INFINITY,-INFINITY,-INFINITY};
  };
  struct RenderData {
    std::vector<Primitive> primitives;
    std::vector<std::vector<uint8_t>> images;
  };

  static std::vector<uint8_t> PNG(int width,int height,const uint8_t* rgba) {
    std::vector<uint8_t> png;
    auto write=[](void* context,void* data,int size) {
      auto& output=*static_cast<std::vector<uint8_t>*>(context);
      auto p=static_cast<uint8_t*>(data); output.insert(output.end(),p,p+size);
    };
    if(!stbi_write_png_to_func(write,&png,width,height,4,rgba,width*4))
      throw std::runtime_error("PNG encoding failed");
    return png;
  }

  // -1: entirely outside, 0: boundary crosses the triangle, 1: entirely inside.
  static int Coverage(const MeshGL64& mesh,size_t face,const ModelLayer& layer) {
    if(layer.repeat) return 1;
    double low[2]={INFINITY,INFINITY}, high[2]={-INFINITY,-INFINITY};
    for(int c=0;c<3;++c) for(int axis=0;axis<2;++axis) {
      double v=mesh.vertProperties[mesh.triVerts[3*face+c]*mesh.numProp+layer.uvIndex+axis];
      low[axis]=std::min(low[axis],v); high[axis]=std::max(high[axis],v);
    }
    const auto& r=layer.coverage;
    if(high[0]<r[0]-1e-9 || high[1]<r[1]-1e-9 || low[0]>r[2]+1e-9 || low[1]>r[3]+1e-9) return -1;
    if(low[0]>=r[0]-1e-9 && low[1]>=r[1]-1e-9 && high[0]<=r[2]+1e-9 && high[1]<=r[3]+1e-9) return 1;
    return 0;
  }

  static void AddPosition(Primitive& primitive,const double* p) {
    for(int axis=0;axis<3;++axis) {
      const float value=float(p[axis]);
      primitive.positions.push_back(value);
      // Bounds describe the serialized float positions, not rounded doubles.
      primitive.low[axis]=std::min(primitive.low[axis],double(value));
      primitive.high[axis]=std::max(primitive.high[axis],double(value));
    }
  }

  static bool Direct(const Model& model,RenderData& data) {
    const int normalIndex=int(model.NumProp());
    auto mesh=model.Geometry().CalculateNormals(normalIndex,55).GetMeshGL64();
    std::map<const ModelImage*,bool> opaque;
    std::map<const ModelImage*,int> imageIndices;
    std::vector<std::shared_ptr<const ModelImage>> images;
    // A material is identified by its original surface and visible layer.
    // Copies of a source solid already have distinct native provenance IDs.
    std::map<std::pair<uint32_t,int>,size_t> groups;
    std::vector<std::unordered_map<uint64_t,uint32_t>> vertices;
    size_t run=0;
    for(size_t face=0;face<mesh.NumTri();++face) {
      while(mesh.runIndex[run+1]<=3*face) ++run;
      const uint32_t id=mesh.runOriginalID[run];
      const auto& appearance=model.Surfaces().at(id);
      int selected=-1;
      // An opaque top layer completely hides lower layers. Outside patches do
      // not affect the material; never infer coverage from just the centroid.
      for(int i=int(appearance.layers.size())-1;i>=0;--i) {
        const auto& layer=appearance.layers[i];
        if(layer.opacity==0) continue;
        int coverage=Coverage(mesh,face,layer);
        if(coverage<0) continue;
        if(coverage==0 || layer.opacity!=1) return false;
        auto found=opaque.find(layer.image.get());
        if(found==opaque.end()) {
          bool solid=true;
          for(size_t p=3;p<layer.image->rgba.size();p+=4)
            if(layer.image->rgba[p]!=255) {solid=false;break;}
          found=opaque.emplace(layer.image.get(),solid).first;
        }
        if(!found->second) return false;
        selected=i;
        break;
      }
      const ModelLayer* layer=selected<0 ? nullptr : &appearance.layers[selected];
      auto inserted=groups.emplace(std::make_pair(id,selected),data.primitives.size());
      if(inserted.second) {
        Primitive primitive;
        if(layer) {
          auto image=imageIndices.emplace(layer->image.get(),int(images.size()));
          if(image.second) images.push_back(layer->image);
          primitive.image=image.first->second;
          primitive.repeat=layer->repeat;
        } else {
          primitive.color=appearance.color;
          primitive.transparent=appearance.color.w<1;
        }
        data.primitives.push_back(std::move(primitive));
        vertices.emplace_back();
      }
      const size_t group=inserted.first->second;
      auto& primitive=data.primitives[group];
      for(int c=0;c<3;++c) {
        const uint64_t vertex=mesh.triVerts[3*face+c];
        auto entry=vertices[group].emplace(vertex,uint32_t(primitive.positions.size()/3));
        primitive.indices.push_back(entry.first->second);
        if(!entry.second) continue;
        const double* p=&mesh.vertProperties[vertex*mesh.numProp];
        AddPosition(primitive,p);
        for(int j=0;j<3;++j) primitive.normals.push_back(float(p[normalIndex+3+j]));
        if(layer) {
          primitive.uv.push_back(float(p[layer->uvIndex]));
          primitive.uv.push_back(float(p[layer->uvIndex+1]));
        } else if(appearance.colorIndex>=0) {
          for(int j=0;j<4;++j) {
            double value=p[appearance.colorIndex+j];
            if(!std::isfinite(value) || value<0 || value>1) return false;
            primitive.colors.push_back(float(value));
            if(j==3 && value<1) primitive.transparent=true;
          }
        }
      }
    }
    // Only encode after classification succeeds. Shared source images are
    // embedded once even when several materials or boolean operands use them.
    for(const auto& image:images) data.images.push_back(PNG(image->width,image->height,image->rgba.data()));
    return true;
  }

  static RenderData Baked(const Model& model,int tileSize) {
    auto baked=model.Bake(tileSize);
    RenderData data;
    Primitive primitive;
    primitive.image=0;
    primitive.indices=std::move(baked.mesh.triVerts);
    for(size_t i=0;i<baked.mesh.NumVert();++i) {
      const float* p=&baked.mesh.vertProperties[8*i];
      double xyz[]={p[0],p[1],p[2]};
      AddPosition(primitive,xyz);
      primitive.uv.insert(primitive.uv.end(),p+3,p+5);
      primitive.normals.insert(primitive.normals.end(),p+5,p+8);
    }
    for(size_t i=3;i<baked.rgba.size();i+=4)
      if(baked.rgba[i]<255) {primitive.transparent=true;break;}
    data.primitives.push_back(std::move(primitive));
    data.images.push_back(PNG(baked.width,baked.height,baked.rgba.data()));
    return data;
  }

  static std::vector<uint8_t> EncodeGLB(const RenderData& data) {
    std::vector<uint8_t> binary;
    struct View {size_t offset,length;};
    struct Accessor {size_t view,count;const char* type;bool indices;};
    struct Attributes {int position,uv,normal,color,index;};
    std::vector<View> views;
    std::vector<Accessor> accessors;
    std::vector<Attributes> attributes;
    auto append=[&](const void* source,size_t length) {
      size_t offset=binary.size(); const auto* p=static_cast<const uint8_t*>(source);
      binary.insert(binary.end(),p,p+length); while(binary.size()%4) binary.push_back(0); return offset;
    };
    auto accessor=[&](const auto& values,int width,const char* type,bool indices=false) {
      if(values.empty()) return -1;
      size_t bytes=values.size()*sizeof(values[0]), offset=append(values.data(),bytes);
      accessors.push_back({views.size(),values.size()/width,type,indices});
      views.push_back({offset,bytes});
      return int(accessors.size()-1);
    };
    for(const auto& p:data.primitives)
      attributes.push_back({accessor(p.positions,3,"VEC3"),accessor(p.uv,2,"VEC2"),
                            accessor(p.normals,3,"VEC3"),accessor(p.colors,4,"VEC4"),
                            accessor(p.indices,1,"SCALAR",true)});
    std::vector<int> positionOwners(accessors.size(),-1);
    for(size_t i=0;i<attributes.size();++i) positionOwners[attributes[i].position]=int(i);
    std::vector<size_t> imageViews;
    for(const auto& image:data.images) {
      imageViews.push_back(views.size());
      views.push_back({append(image.data(),image.size()),image.size()});
    }
    std::vector<int> textures(data.primitives.size(),-1);
    int textureCount=0;
    for(size_t i=0;i<data.primitives.size();++i) if(data.primitives[i].image>=0) textures[i]=textureCount++;
    std::ostringstream json; json.imbue(std::locale::classic()); json.precision(17);
    json<<"{\"asset\":{\"version\":\"2.0\",\"generator\":\"manifold::Model\"},\"scene\":0,\"scenes\":[{\"nodes\":[0]}],\"nodes\":[{\"mesh\":0}],"
      "\"meshes\":[{\"primitives\":[";
    for(size_t i=0;i<attributes.size();++i) {
      if(i) json<<",";
      const auto& a=attributes[i];
      json<<"{\"attributes\":{\"POSITION\":"<<a.position<<",\"NORMAL\":"<<a.normal;
      if(a.uv>=0) json<<",\"TEXCOORD_0\":"<<a.uv;
      if(a.color>=0) json<<",\"COLOR_0\":"<<a.color;
      json<<"},\"indices\":"<<a.index<<",\"material\":"<<i<<"}";
    }
    json<<"]}],\"materials\":[";
    for(size_t i=0;i<data.primitives.size();++i) {
      if(i) json<<",";
      const auto& p=data.primitives[i];
      json<<"{\"alphaMode\":\""<<(p.transparent?"BLEND":"OPAQUE")<<"\",\"pbrMetallicRoughness\":{\"metallicFactor\":0,\"roughnessFactor\":0.8,"
          "\"baseColorFactor\":["<<p.color.x<<","<<p.color.y<<","<<p.color.z<<","<<p.color.w<<"]";
      if(textures[i]>=0) json<<",\"baseColorTexture\":{\"index\":"<<textures[i]<<"}";
      json<<"}}";
    }
    json<<"]";
    if(textureCount) {
      json<<",\"textures\":[";
      bool comma=false;
      for(const auto& p:data.primitives) if(p.image>=0) {
        if(comma) json<<",";
        comma=true;
        json<<"{\"sampler\":"<<(p.repeat?1:0)<<",\"source\":"<<p.image<<"}";
      }
      json<<"],\"samplers\":[{\"magFilter\":9729,\"minFilter\":9729,\"wrapS\":33071,\"wrapT\":33071},"
            "{\"magFilter\":9729,\"minFilter\":9729,\"wrapS\":10497,\"wrapT\":10497}],\"images\":[";
      for(size_t i=0;i<imageViews.size();++i) {
        if(i) json<<",";
        json<<"{\"bufferView\":"<<imageViews[i]<<",\"mimeType\":\"image/png\"}";
      }
      json<<"]";
    }
    json<<",\"buffers\":[{\"byteLength\":"<<binary.size()<<"}],\"bufferViews\":[";
    for(size_t i=0;i<views.size();++i) {
      if(i) json<<",";
      json<<"{\"buffer\":0,\"byteOffset\":"<<views[i].offset<<",\"byteLength\":"<<views[i].length<<"}";
    }
    json<<"],\"accessors\":[";
    for(size_t i=0;i<accessors.size();++i) {
      if(i) json<<",";
      const auto& a=accessors[i];
      json<<"{\"bufferView\":"<<a.view<<",\"componentType\":"<<(a.indices?5125:5126)<<",\"count\":"<<a.count<<",\"type\":\""<<a.type<<"\"";
      if(positionOwners[i]>=0) {
        const auto& primitive=data.primitives[positionOwners[i]];
        const auto& low=primitive.low;const auto& high=primitive.high;
        json<<",\"min\":["<<low.x<<","<<low.y<<","<<low.z<<"],\"max\":["<<high.x<<","<<high.y<<","<<high.z<<"]";
      }
      json<<"}";
    }
    json<<"]}";
    std::string doc=json.str(); while(doc.size()%4) doc.push_back(' ');
    std::vector<uint8_t> result;
    auto word=[&](uint32_t v) { for(int i=0;i<4;++i) result.push_back(uint8_t(v>>(8*i))); };
    word(0x46546c67); word(2); word(uint32_t(28+doc.size()+binary.size()));
    word(uint32_t(doc.size())); word(0x4e4f534a); result.insert(result.end(),doc.begin(),doc.end());
    word(uint32_t(binary.size())); word(0x004e4942); result.insert(result.end(),binary.begin(),binary.end());
    return result;
  }
};
} // namespace manifold
