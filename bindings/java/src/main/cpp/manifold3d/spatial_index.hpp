#pragma once
#include "manifold/manifold.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <numeric>
#include <queue>
#include <stdexcept>
#include <vector>

namespace MeshUtils {
// Owns an immutable snapshot. Triangle IDs refer to GetMeshGL triangle order.
class SpatialIndex {
  using V = manifold::vec3;
  struct Bounds {
    V lo={INFINITY,INFINITY,INFINITY}, hi={-INFINITY,-INFINITY,-INFINITY};
    void add(V p) { lo=linalg::min(lo,p); hi=linalg::max(hi,p); }
    bool overlap(const Bounds& b,double e) const {
      for(int k=0;k<3;++k) if(hi[k]+e<b.lo[k] || b.hi[k]+e<lo[k]) return false;
      return true;
    }
    double distance2(V p) const {
      V d=linalg::max(linalg::max(lo-p,p-hi),V(0.0)); return linalg::dot(d,d);
    }
    bool ray(V o,V d,double limit) const {
      double a=0,b=limit;
      for(int k=0;k<3;++k) {
        if(std::abs(d[k])<1e-30) { if(o[k]<lo[k] || o[k]>hi[k]) return false; }
        else { double u=(lo[k]-o[k])/d[k],v=(hi[k]-o[k])/d[k]; if(u>v) std::swap(u,v); a=std::max(a,u); b=std::min(b,v); }
      }
      return a<=b;
    }
  };
  struct Tri { V a,b,c; Bounds box; };
  struct Node { Bounds box; int begin=0,end=0,left=-1,right=-1; };
  std::vector<Tri> tris;
  std::vector<int> order;
  std::vector<Node> nodes;
  std::vector<V> shellPoints;
  int build(int begin,int end) {
    Node n; n.begin=begin; n.end=end;
    for(int i=begin;i<end;++i) { n.box.add(tris[order[i]].box.lo); n.box.add(tris[order[i]].box.hi); }
    int id=nodes.size(); nodes.push_back(n);
    if(end-begin>8) {
      V size=n.box.hi-n.box.lo; int axis=size.x>size.y ? 0:1; if(size.z>size[axis]) axis=2;
      int mid=(begin+end)/2;
      std::nth_element(order.begin()+begin,order.begin()+mid,order.begin()+end,[&](int a,int b) {
        return tris[a].box.lo[axis]+tris[a].box.hi[axis]<tris[b].box.lo[axis]+tris[b].box.hi[axis];
      });
      int l=build(begin,mid),r=build(mid,end); nodes[id].left=l; nodes[id].right=r;
    }
    return id;
  }
  static double dot(V a,V b) { return linalg::dot(a,b); }
  static V cross(V a,V b) { return linalg::cross(a,b); }
  static double length(V a) { return std::sqrt(dot(a,a)); }
  static void finite(V a) { for(int i=0;i<3;++i) if(!std::isfinite(a[i])) throw std::invalid_argument("Query coordinates must be finite"); }
  static void epsilon(double e) { if(!std::isfinite(e)||e<0) throw std::invalid_argument("Tolerance must be finite and nonnegative"); }
  static bool hit(const Tri& t,V o,V d,double& distance,double& u,double& v) {
    V e=t.b-t.a,f=t.c-t.a,p=cross(d,f); double det=dot(e,p);
    if(std::abs(det)<=1e-14*length(e)*length(f)) return false;
    V q=o-t.a; u=dot(q,p)/det; V r=cross(q,e); v=dot(d,r)/det; distance=dot(f,r)/det;
    return u>=-1e-12 && v>=-1e-12 && u+v<=1+1e-12 && distance>=0;
  }
  static V closest(const Tri& t,V p) {
    // Voronoi regions of a triangle, including its three edges and corners.
    V ab=t.b-t.a,ac=t.c-t.a,ap=p-t.a;
    double d1=dot(ab,ap),d2=dot(ac,ap); if(d1<=0&&d2<=0) return t.a;
    V bp=p-t.b; double d3=dot(ab,bp),d4=dot(ac,bp); if(d3>=0&&d4<=d3) return t.b;
    double vc=d1*d4-d3*d2; if(vc<=0&&d1>=0&&d3<=0) return t.a+(d1/(d1-d3))*ab;
    V cp=p-t.c; double d5=dot(ab,cp),d6=dot(ac,cp); if(d6>=0&&d5<=d6) return t.c;
    double vb=d5*d2-d1*d6; if(vb<=0&&d2>=0&&d6<=0) return t.a+(d2/(d2-d6))*ac;
    double va=d3*d6-d5*d4; if(va<=0&&d4-d3>=0&&d5-d6>=0) return t.b+((d4-d3)/((d4-d3)+(d5-d6)))*(t.c-t.b);
    return t.a+(vb*ab+vc*ac)/(va+vb+vc);
  }
  static bool intersects(const Tri& a,const Tri& b,double e) {
    std::array<V,3> av={a.a,a.b,a.c},bv={b.a,b.b,b.c};
    std::array<V,3> ae={a.b-a.a,a.c-a.b,a.a-a.c},be={b.b-b.a,b.c-b.b,b.a-b.c};
    V an=cross(ae[0],ae[1]),bn=cross(be[0],be[1]);
    auto separated=[&](V axis) {
      double len=length(axis); if(len<1e-30) return false; axis=axis/len;
      double al=INFINITY,ah=-INFINITY,bl=INFINITY,bh=-INFINITY;
      for(V v:av) { double d=dot(v-a.a,axis); al=std::min(al,d); ah=std::max(ah,d); }
      for(V v:bv) { double d=dot(v-a.a,axis); bl=std::min(bl,d); bh=std::max(bh,d); }
      return ah+e<bl || bh+e<al;
    };
    if(separated(an)||separated(bn)) return false;
    for(V x:ae) for(V y:be) if(separated(cross(x,y))) return false;
    // Additional in-plane axes also separate coplanar triangles.
    for(V x:ae) if(separated(cross(an,x))) return false;
    for(V y:be) if(separated(cross(bn,y))) return false;
    return true;
  }
 public:
  explicit SpatialIndex(const manifold::Manifold& solid) {
    if(solid.Status()!=manifold::Manifold::Error::NoError) throw std::invalid_argument("Spatial index requires a valid Manifold");
    auto mesh=solid.GetMeshGL64();
    // One containment probe per connected shell, joining UV/normal seam copies.
    std::vector<size_t> parent(mesh.NumVert());
    std::iota(parent.begin(),parent.end(),0);
    auto root=[&](size_t v) { while(parent[v]!=v) { parent[v]=parent[parent[v]]; v=parent[v]; } return v; };
    auto join=[&](size_t a,size_t b) { parent[root(a)]=root(b); };
    for(size_t i=0;i<mesh.mergeFromVert.size();++i) join(mesh.mergeFromVert[i],mesh.mergeToVert[i]);
    for(size_t i=0;i<mesh.NumTri();++i) {
      join(mesh.triVerts[i*3],mesh.triVerts[i*3+1]); join(mesh.triVerts[i*3],mesh.triVerts[i*3+2]);
      Tri t; t.a=mesh.GetVertPos(mesh.triVerts[i*3]); t.b=mesh.GetVertPos(mesh.triVerts[i*3+1]); t.c=mesh.GetVertPos(mesh.triVerts[i*3+2]);
      t.box.add(t.a); t.box.add(t.b); t.box.add(t.c); tris.push_back(t);
    }
    std::vector<bool> seen(parent.size(),false);
    for(size_t i=0;i<tris.size();++i) { size_t r=root(mesh.triVerts[i*3]); if(!seen[r]) { seen[r]=true; shellPoints.push_back(tris[i].a); } }
    order.resize(tris.size()); std::iota(order.begin(),order.end(),0); if(!tris.empty()) build(0,tris.size());
  }
  // Results: triangle, distance, position XYZ, normal XYZ, barycentric UV.
  std::vector<double> RayCast(double ox,double oy,double oz,double dx,double dy,double dz,double limit) const {
    V o={ox,oy,oz},d={dx,dy,dz}; finite(o); finite(d);
    double len=std::hypot(dx,dy,dz); if(!std::isfinite(len) || len==0 || std::isnan(limit)||limit<0) throw std::invalid_argument("Ray needs nonzero finite direction and nonnegative distance"); d=d/len;
    std::vector<double> out; std::vector<int> stack; if(!nodes.empty()) stack.push_back(0);
    while(!stack.empty()) { int id=stack.back(); stack.pop_back(); auto& n=nodes[id]; if(!n.box.ray(o,d,limit)) continue;
      if(n.left>=0) { stack.push_back(n.left); stack.push_back(n.right); continue; }
      for(int i=n.begin;i<n.end;++i) { int ti=order[i]; auto& t=tris[ti]; double dist,u,v;
        if(hit(t,o,d,dist,u,v)&&dist<=limit) { limit=dist; V p=o+dist*d,normal=cross(t.b-t.a,t.c-t.a); normal=normal/length(normal);
          out={double(ti),dist,p.x,p.y,p.z,normal.x,normal.y,normal.z,u,v}; }
      }
    }
    return out;
  }
  std::vector<double> ClosestPoint(double x,double y,double z) const {
    V p={x,y,z}; finite(p); double best=INFINITY; int tri=-1; V point;
    using Q=std::pair<double,int>; std::priority_queue<Q,std::vector<Q>,std::greater<Q>> queue;
    if(!nodes.empty()) queue.push({nodes[0].box.distance2(p),0});
    while(!queue.empty()) { auto [distance,id]=queue.top(); queue.pop(); if(distance>best) break; auto& n=nodes[id];
      if(n.left>=0) { for(int c:{n.left,n.right}) queue.push({nodes[c].box.distance2(p),c}); continue; }
      for(int i=n.begin;i<n.end;++i) { int t=order[i]; V q=closest(tris[t],p); double d=dot(q-p,q-p); if(d<best) { best=d; tri=t; point=q; } }
    }
    if(tri<0) return {};
    auto& t=tris[tri]; V normal=cross(t.b-t.a,t.c-t.a); normal=normal/length(normal);
    return {double(tri),std::sqrt(best),point.x,point.y,point.z,normal.x,normal.y,normal.z};
  }
  // -1 outside, 0 boundary, 1 inside; parity includes enclosed cavities.
  int ClassifyPoint(double x,double y,double z,double tolerance) const {
    epsilon(tolerance); V p={x,y,z}; finite(p); if(nodes.empty()) return -1;
    auto near=ClosestPoint(x,y,z); if(near[1]<=tolerance) return 0;
    int votes=0;
    for(V d:{V{1,.371,.217},V{.193,1,.437},V{.317,.239,1}}) {
      d=d/length(d); std::vector<double> distances; std::vector<int> stack={0};
      while(!stack.empty()) { int id=stack.back(); stack.pop_back(); auto& n=nodes[id]; if(!n.box.ray(p,d,INFINITY)) continue;
        if(n.left>=0) { stack.push_back(n.left); stack.push_back(n.right); continue; }
        for(int i=n.begin;i<n.end;++i) { double t,u,v; if(hit(tris[order[i]],p,d,t,u,v)) distances.push_back(t); }
      }
      std::sort(distances.begin(),distances.end()); size_t count=0; double last=-INFINITY;
      for(double t:distances) if(t-last>1e-10*std::max(1.0,std::abs(t))) { ++count; last=t; }
      votes+=count%2;
    }
    return votes>=2 ? 1:-1;
  }
  bool Overlaps(const SpatialIndex& other,double tolerance) const {
    epsilon(tolerance); if(nodes.empty()||other.nodes.empty()) return false;
    std::vector<std::pair<int,int>> stack={{0,0}};
    while(!stack.empty()) { auto [a,b]=stack.back(); stack.pop_back(); auto& na=nodes[a]; auto& nb=other.nodes[b];
      if(!na.box.overlap(nb.box,tolerance)) continue;
      if(na.left>=0) { stack.push_back({na.left,b}); stack.push_back({na.right,b}); }
      else if(nb.left>=0) { stack.push_back({a,nb.left}); stack.push_back({a,nb.right}); }
      else for(int i=na.begin;i<na.end;++i) for(int j=nb.begin;j<nb.end;++j)
        if(intersects(tris[order[i]],other.tris[other.order[j]],tolerance)) return true;
    }
    for(V p:shellPoints) if(other.nodes[0].box.distance2(p)==0 && other.ClassifyPoint(p.x,p.y,p.z,tolerance)>=0) return true;
    for(V p:other.shellPoints) if(nodes[0].box.distance2(p)==0 && ClassifyPoint(p.x,p.y,p.z,tolerance)>=0) return true;
    return false;
  }
};
} // namespace MeshUtils
