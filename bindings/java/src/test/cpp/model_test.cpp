#include "model_utils.hpp"
#include <iostream>
using namespace manifold;
void check(bool condition,const char* message) { if(!condition) throw std::runtime_error(message); }
std::shared_ptr<const ModelImage> image(uint8_t r,uint8_t g,uint8_t b,uint8_t a=255) {
  return std::make_shared<const ModelImage>(1,1,std::vector<uint8_t>{r,g,b,a});
}
Model::Mapping planar=[](const Manifold& m,int channel) {
  return MeshUtils::ApplyPlanarUV(m,channel,0,2,0.2,0.2,0.5,0.5);
};
int embeddedImageWidth(const std::vector<uint8_t>& bytes) {
  const std::vector<uint8_t> signature{137,80,78,71,13,10,26,10};
  auto at=std::search(bytes.begin(),bytes.end(),signature.begin(),signature.end());
  check(at!=bytes.end(),"GLB embeds a PNG");
  int w=0,h=0,n=0;
  check(stbi_info_from_memory(&*at,int(bytes.end()-at),&w,&h,&n),"embedded PNG header is valid");
  return w;
}

void wholeSurfaceUV() {
  const auto source=Manifold::Cube({4,6,8},true).Translate({1000000000.125,0,0});
  const auto mapped=MeshUtils::ApplyBoxUV(source,3,1000000000.125,0,0,2,3,1,1,0,0);
  check(mapped.Status()==Manifold::Error::NoError,"box UV keeps manifold topology");
  check(mapped.NumVert()==source.NumVert() && mapped.NumTri()==source.NumTri(),"only property vertices split");
  check(mapped.Volume()==source.Volume(),"box UV preserves double precision geometry");
  const auto mesh=mapped.GetMeshGL64();
  check(!mesh.mergeFromVert.empty(),"box seams retain physical merge metadata");
  for(size_t f=0;f<mesh.NumTri();++f) {
    const double* a=&mesh.vertProperties[mesh.numProp*mesh.triVerts[3*f]];
    const double* b=&mesh.vertProperties[mesh.numProp*mesh.triVerts[3*f+1]];
    const double* c=&mesh.vertProperties[mesh.numProp*mesh.triVerts[3*f+2]];
    check(a[0]==1000000000.125-2 || a[0]==1000000000.125+2,"box projection does not round positions through float");
    check(std::abs((b[3]-a[3])*(c[4]-a[4])-(b[4]-a[4])*(c[3]-a[3]))>0,"every box face has UV area");
  }
  const auto twice=MeshUtils::ApplyBoxUV(mapped,5,0,0,0,4,4,-1,1,0.25,0.5).GetMeshGL64();
  for(size_t v=0;v<twice.NumVert();++v) {
    const auto p=&twice.vertProperties[v*twice.numProp];
    bool found=false;
    for(size_t old=0;old<mesh.NumVert();++old)
      found |= std::equal(p,p+5,&mesh.vertProperties[old*mesh.numProp]);
    check(found,"adding whole-surface UVs retains previous vertex properties");
  }
  for(double size:{0.0,-1.0,std::numeric_limits<double>::infinity()}) {
    try { MeshUtils::ApplyBoxUV(source,3,0,0,0,size,1,1,1,0,0); check(false,"invalid box size accepted"); }
    catch(const std::invalid_argument&) {}
  }
}
int main() {
  try {
    wholeSurfaceUV();
    Model original(Manifold::Cube({4,4,4},true));
    auto red=original.Texture(image(255,0,0),planar,"red");
    auto layered=red.Texture(image(0,0,255,128),planar,"blue");
    check(original.LayerCount()==0 && red.LayerCount()==1 && layered.LayerCount()==2,"immutable layers");
    auto color=layered.SampleColor(0,0.2,0.3);
    check(std::abs(color.x-127.0/255)<1e-8 && std::abs(color.z-128.0/255)<1e-8,"source-over alpha");
    check(red.GetMeshGL64().numProp==5 && layered.GetMeshGL64().numProp==7,"UV channel allocation");
    auto blue=original.Texture(image(0,0,255),planar,"blue").Translate({1,0.5,0.25});
    for(auto op:{OpType::Add,OpType::Subtract,OpType::Intersect}) {
      auto result=red.Boolean(blue,op);
      bool r=false,b=false;
      for(size_t f=0;f<result.NumTri();++f) { auto c=result.SampleColor(f,0.2,0.3); r|=c.x>0.99; b|=c.z>0.99; }
      check(r && b,"boolean preserves independent appearances derived from same original");
      check(result.LayerCount()==2,"boolean retains assets");
    }
    auto moved=layered.Translate({1,2,3}).Rotate(10,20,30).Refine(2);
    auto joined=red.Compose(blue.Translate({6,0,0}));
    check(joined.Decompose().size()==2,"compose/decompose geometry");
    for(const auto& part:joined.Decompose()) {
      check(part.ImageCount()==1,"empty provenance runs cannot retain absent images");
      check(part.Color({1,1,1,1}).ImageCount()==1,"reidentifying decomposed geometry preserves surviving layers");
    }
    check(moved.LayerCount()==2 && moved.ImageCount()==2,"transform/refine preserve layers");
    auto baked=moved.Bake(4);
    check(Manifold(baked.mesh).Status()==Manifold::Error::NoError,"render mesh physical connectivity");
    for(size_t face=0;face<baked.mesh.NumTri();++face) {
      std::array<vec3,3> xyz;
      std::array<vec2,3> uv;
      for(int c=0;c<3;++c) {
        const float* p=&baked.mesh.vertProperties[8*baked.mesh.triVerts[face*3+c]];
        xyz[c]={p[0],p[1],p[2]}; uv[c]={p[3]*baked.width,p[4]*baked.height};
      }
      double density=la::length(uv[1]-uv[0])/la::length(xyz[1]-xyz[0]);
      for(int c=1;c<3;++c)
        check(std::abs(la::length(uv[(c+1)%3]-uv[c])-density*la::length(xyz[(c+1)%3]-xyz[c]))<0.001,
              "atlas preserves triangle shape to avoid MSAA extrapolation on slivers");
    }
    auto bytes=ModelIO::ExportGLB(layered,8);
    check(bytes.size()>100 && bytes[0]=='g' && bytes[1]=='l',"native GLB export");
    check(embeddedImageWidth(bytes)>1,"transparent layering falls back to the compositor");
    check(embeddedImageWidth(ModelIO::ExportGLB(red))==1,"opaque image exports at source resolution");
    auto crossing=original.Texture(image(255,0,0),planar,"partial",1,false,{0.3,0.3,0.7,0.7});
    check(embeddedImageWidth(ModelIO::ExportGLB(crossing,4))>1,"crossing coverage cannot be approximated by a face centroid");
    auto hidden=layered.Texture(image(0,255,0),planar,"opaque top");
    check(embeddedImageWidth(ModelIO::ExportGLB(hidden))==1,"opaque top layer hides lower alpha layers");
    check(hidden.LayerCount()==3,"direct export does not discard hidden native layers");
    auto opacity=original.Texture(image(255,0,0),planar,"opacity",0.5);
    check(embeddedImageWidth(ModelIO::ExportGLB(opacity,4))>1,"layer opacity requires compositing");
    auto invalid=[](const Manifold& g,int) { return g; };
    try { original.Texture(image(1,2,3),invalid); check(false,"missing UV should fail"); } catch(const std::invalid_argument&) {}
    check(original.LayerCount()==0,"failure cannot change input");
    std::cout<<"Model: immutable layers, alpha compositing, UV allocation, boolean provenance, transforms, atlas topology and GLB passed\n";
  } catch(const std::exception& e) { std::cerr<<e.what()<<"\n"; return 1; }
}
