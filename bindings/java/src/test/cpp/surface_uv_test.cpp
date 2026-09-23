#include "mesh_utils.hpp"
#include <iostream>

using manifold::Manifold;

Manifold Map(const Manifold& source, double width, double height, double step,
             MeshUtils::vec3 origin={0,0,10}, MeshUtils::vec3 normal={0,0,1},
             MeshUtils::vec3 right={1,0,0}) {
    return MeshUtils::GeodesicUV(source,3,origin.x,origin.y,origin.z,
                                 normal.x,normal.y,normal.z,right.x,right.y,right.z,
                                 width,height,0.25,1.0/3,0.5,1.0/3,0.05,0.05,step);
}

void Check(bool condition, const char* description) {
    if (!condition) throw std::runtime_error(description);
}

double UVArea(const manifold::MeshGL64& mesh) {
    double area=0;
    for(size_t f=0;f<mesh.NumTri();++f) {
        MeshUtils::vec2 uv[3];
        for(int k=0;k<3;++k) {
            size_t offset=mesh.triVerts[3*f+k]*mesh.numProp;
            uv[k]={mesh.vertProperties[offset+3],mesh.vertProperties[offset+4]};
        }
        area+=std::abs(MeshUtils::SurfaceUV::Cross2(uv[1]-uv[0],uv[2]-uv[0]))/2;
    }
    return area;
}

Manifold Torus() {
    manifold::MeshGL64 mesh;
    constexpr int major=64,minor=32;
    const double pi=std::acos(-1.0);
    for(int i=0;i<major;++i) for(int j=0;j<minor;++j) {
        double a=2*pi*i/major,b=2*pi*j/minor;
        mesh.vertProperties.insert(mesh.vertProperties.end(),
            {(7+3*std::cos(b))*std::cos(a),(7+3*std::cos(b))*std::sin(a),3*std::sin(b)});
        uint64_t p=i*minor+j,q=((i+1)%major)*minor+j,
                 r=((i+1)%major)*minor+(j+1)%minor,s=i*minor+(j+1)%minor;
        mesh.triVerts.insert(mesh.triVerts.end(),{p,q,r,p,r,s});
    }
    return Manifold(mesh);
}

void CheckShape(const char* name,const Manifold& source,MeshUtils::vec3 origin,
                MeshUtils::vec3 normal,MeshUtils::vec3 right,double width,double height) {
    Manifold mapped=Map(source,width,height,0.13,origin,normal,right);
    Check(std::abs(mapped.Volume()-source.Volume())<1e-6,"shape changed volume");
    Check(std::abs(mapped.SurfaceArea()-source.SurfaceArea())<1e-6,"shape changed area");
    Check(mapped.Genus()==source.Genus(),"shape changed genus");
    Check(std::abs(UVArea(mapped.GetMeshGL64())-1.0/6)<1e-7,"incomplete or overlapping texture");
    std::cout<<name<<": "<<source.NumTri()<<" -> "<<mapped.NumTri()<<" triangles"<<std::endl;
}

void CheckDepth(const char* name,const Manifold& source,MeshUtils::vec3 origin,
                MeshUtils::vec3 normal,MeshUtils::vec3 right,
                const std::function<MeshUtils::vec3(MeshUtils::vec3)>& expectedNormal) {
    using namespace MeshUtils::SurfaceUV;
    Topology topology(source);
    Grid grid(topology,origin,normal,right,2.4,1.6,0.2);
    auto output=[&](const DepthField& depth) {
        Remesh remesh(topology,grid);
        remesh.Build();
        return remesh.Output(3,{0,0},{1,1},{-1,-1},depth);
    };
    auto flat=output({});
    auto zero=output(DepthField({0,0,0,0},2,2,1,0,0));
    Check(flat.vertProperties==zero.vertProperties && flat.triVerts==zero.triVerts,
          "zero depth changed geometry or UVs");
    for (double sign:{-1.0,1.0}) for (bool step:{false,true}) {
        DepthField depth({1,1,1,1},2,2,sign*0.15,0,step ? 0 : 0.4,step);
        auto displaced=output(depth);
        Check(step || displaced.triVerts==flat.triVerts,"depth changed mapped topology");
        size_t moved=0;
        for(size_t i=0;i<flat.NumVert();++i) {
            Point p=flat.GetVertPos(i), d=displaced.GetVertPos(i)-p;
            Point2 uv{flat.vertProperties[5*i+3],flat.vertProperties[5*i+4]};
            Check(uv.x==displaced.vertProperties[5*i+3] && uv.y==displaced.vertProperties[5*i+4],
                  "depth changed UVs");
            double expected=uv.x<0 ? 0 : depth.Evaluate(uv,2.4,1.6,topology.epsilon);
            Check(std::abs(linalg::length(d)-std::abs(expected))<1e-9,"wrong displacement magnitude");
            if (std::abs(expected)>1e-6) {
                ++moved;
                Check(linalg::dot(d/expected,expectedNormal(p))>0.995,"depth is not along the local outward normal");
            }
        }
        Check(moved>20,"no depth samples moved");
        for (size_t i=0;i<displaced.mergeFromVert.size();++i)
            Check(linalg::length(displaced.GetVertPos(displaced.mergeFromVert[i])-
                                  displaced.GetVertPos(displaced.mergeToVert[i]))==0,
                  "depth split a physical UV seam");
        Manifold result(displaced);
        Check(result.Status()==Manifold::Error::NoError && !result.IsEmpty(),"invalid depth manifold");
        Check(result.Genus()==source.Genus(),"depth changed genus");
        Check(sign*(result.Volume()-source.Volume())>0,"signed depth has wrong volume change");
        Manifold cut=result-Manifold::Cube({0.8,0.8,4},true).Translate(origin);
        Check(cut.Status()==Manifold::Error::NoError && !cut.IsEmpty(),"boolean after depth failed");
    }
    std::cout<<name<<": fade/step, outward/inward depth, unchanged UVs, exact seam merges and booleans verified"<<std::endl;
}

void CheckMiter(const char* name,const Manifold& source,MeshUtils::vec3 origin,
                MeshUtils::vec3 normal,MeshUtils::vec3 right,MeshUtils::vec3 direction) {
    using namespace MeshUtils::SurfaceUV;
    Topology topology(source);
    Grid grid(topology,origin,normal,right,3,2,0.2);
    auto output=[&](const DepthField& depth) {
        Remesh remesh(topology,grid);
        remesh.Build();
        return remesh.Output(3,{0,0},{1,1},{-1,-1},depth);
    };
    auto flat=output({});
    auto key=[](double u,double v) { return std::make_pair(std::llround(u*1e8),std::llround(v*1e8)); };
    std::map<std::pair<int64_t,int64_t>,Point> samples;
    for (size_t i=0;i<flat.NumVert();++i)
        if (flat.vertProperties[5*i+3]>=0)
            samples[key(flat.vertProperties[5*i+3],flat.vertProperties[5*i+4])]=flat.GetVertPos(i);
    double area=0;
    for (size_t f=0;f<flat.NumTri();++f) {
        auto a=flat.triVerts[3*f],b=flat.triVerts[3*f+1],c=flat.triVerts[3*f+2];
        if (flat.vertProperties[5*a+3]>=0)
            area+=linalg::length(linalg::cross(flat.GetVertPos(b)-flat.GetVertPos(a),
                                              flat.GetVertPos(c)-flat.GetVertPos(a)))/2;
    }
    for (double amount:{-0.15,0.0,0.01,0.3,1.0}) {
        auto mesh=output(DepthField({1,1,1,1},2,2,amount,0,0,true));
        for (size_t i=0;i<mesh.NumVert();++i) {
            double u=mesh.vertProperties[5*i+3],v=mesh.vertProperties[5*i+4];
            if (u<0) continue;
            auto original=samples.find(key(u,v));
            Check(original!=samples.end(),"miter changed the UV sample coordinates");
            Check(linalg::length(mesh.GetVertPos(i)-original->second-amount*direction)<1e-7,
                  "miter does not satisfy the incident offset planes");
        }
        for (size_t i=0;i<mesh.mergeFromVert.size();++i)
            Check(linalg::length(mesh.GetVertPos(mesh.mergeFromVert[i])-mesh.GetVertPos(mesh.mergeToVert[i]))==0,
                  "miter split a physical seam");
        Check(std::abs(UVArea(mesh)-1)<1e-8,"miter lost or overlapped part of the UV chart");
        Manifold result(mesh);
        Check(result.Status()==Manifold::Error::NoError && !result.IsEmpty(),"invalid miter manifold");
        Check(result.Genus()==source.Genus(),"miter changed topology");
        Check(std::abs(result.Volume()-source.Volume()-area*amount)<1e-6,"wrong miter swept volume");
        Manifold cut=result-Manifold::Cube({0.8,0.8,8},true).Translate(origin+Point{0.7,0.7,0.7});
        Check(cut.Status()==Manifold::Error::NoError && !cut.IsEmpty(),"boolean after miter failed");
    }
    // Nonuniform signed depth and a fade use the same continuous miter field.
    for (const auto& samplesGrid : {std::vector<double>{1,1,1,1,2,1,1,1,1},
                                   std::vector<double>{0,0,1,0,1,1,0,0,1}})
    for (double sign:{-1.0,1.0}) for (bool step:{false,true}) {
        DepthField depth(samplesGrid,3,3,sign*0.03,0,step ? 0 : 0.4,step);
        auto mesh=output(depth);
        for (size_t i=0;i<mesh.NumVert();++i) {
            double u=mesh.vertProperties[5*i+3],v=mesh.vertProperties[5*i+4];
            if (u<0) continue;
            double d=depth.Evaluate({u,v},3,2,topology.epsilon);
            Check(linalg::length(mesh.GetVertPos(i)-samples.at(key(u,v))-d*direction)<1e-7,
                  "nonuniform miter depth is incorrect");
        }
        Manifold result(mesh);
        Check(result.Status()==Manifold::Error::NoError && result.Genus()==source.Genus(),"invalid varying miter depth");
    }
    std::cout<<name<<": exact offset planes, UV coverage, signed volume, variable depth and booleans verified"<<std::endl;
}

int main() {
    try {
        for (int resolution : {16,32,64,96}) {
            Manifold sphere=Manifold::Sphere(10,resolution);
            Manifold mapped=Map(sphere,6,4,0.2);
            Check(std::abs(mapped.Volume()-sphere.Volume())<1e-6,"changed volume");
            Check(std::abs(mapped.SurfaceArea()-sphere.SurfaceArea())<1e-6,"changed area");
            Check(mapped.Genus()==0,"changed genus");
            auto mesh=mapped.GetMeshGL64();
            size_t inside=0, outside=0;
            for (size_t f=0;f<mesh.NumTri();++f) {
                int background=0;
                for (int c=0;c<3;++c) {
                    size_t off=mesh.triVerts[3*f+c]*mesh.numProp;
                    double u=mesh.vertProperties[off+3],v=mesh.vertProperties[off+4];
                    bool out=std::abs(u-0.05)<1e-7 && std::abs(v-0.05)<1e-7;
                    background+=out;
                    Check(out || (u>=0.25-1e-7 && u<=0.75+1e-7 && v>=1.0/3-1e-7 && v<=2.0/3+1e-7),"UV outside atlas");
                    if (!out) Check(mesh.vertProperties[off+2]>8,"mapped back of sphere");
                }
                Check(background==0 || background==3,"triangle crosses UV seam");
                background ? ++outside : ++inside;
            }
            Check(inside>0 && outside>0,"missing patch or background");
            Check(std::abs(UVArea(mesh)-1.0/6)<1e-7,"incomplete sphere texture");
            std::cout<<"sphere "<<resolution<<": "<<sphere.NumTri()<<" -> "<<mapped.NumTri()<<" triangles; patch "<<inside<<std::endl;
            Manifold cut=Manifold::Cube({2,2,6},true).Translate({2,0,9});
            Manifold boolean=mapped-cut;
            Check(boolean.Status()==Manifold::Error::NoError && !boolean.IsEmpty(),"boolean failed");
            Check(std::abs(boolean.Volume()-(sphere-cut).Volume())<1e-6,"boolean changed volume");
        }
        Manifold cube=Manifold::Cube({10,10,2},true);
        Manifold mapped=Map(cube,3,2,0.25,{0.31,0.17,1});
        Check(std::abs(mapped.Volume()-cube.Volume())<1e-7,"flat interior seed changed volume");
        Check(mapped.Genus()==0,"flat patch is not closed");
        std::cout<<"flat interior seed: "<<mapped.NumTri()<<" triangles"<<std::endl;
        Manifold flat=MeshUtils::GeodesicUV(cube,3,0.31,0.17,1,0,0,1,1,0,0,3,2,0,0,1,1,-1,-1,0.25);
        auto flatMesh=flat.GetMeshGL64();
        Check(std::abs(UVArea(flatMesh)-1)<1e-9,"flat sticker corner was clipped");
        // All 117 samples must exist, even though the original box face only
        // has two triangles. This exercises face-interior insertion, crossing
        // the diagonal through paired halfedges, and shared edge insertion.
        for(int x=0;x<=12;++x) for(int y=0;y<=8;++y) {
            bool found=false;
            MeshUtils::vec3 expected{-1.19+0.25*x,-0.83+0.25*y,1};
            for(size_t i=0;i<flatMesh.NumVert();++i) {
                auto p=flatMesh.GetVertPos(i);
                if(linalg::length(p-expected)<1e-8 &&
                   std::abs(flatMesh.vertProperties[5*i+3]-x/12.0)<1e-8 &&
                   std::abs(flatMesh.vertProperties[5*i+4]-(1-y/8.0))<1e-8) found=true;
            }
            Check(found,"missing pixel sample or wrong walked distance");
        }
        std::cout<<"all 117 planar samples and complete UV coverage verified"<<std::endl;
        CheckShape("ellipsoid",Manifold::Sphere(10,48).Scale({1.6,0.7,1.1}),
                   {2.0,0.7,11},{0,0,1},{1,0.3,0},5.2,3.3);
        CheckShape("cylinder",Manifold::Cylinder(10,5,5,64),
                   {5,0,5},{1,0,0},{0,1,0},5.3,3.1);
        CheckShape("torus outer",Torus(),{10,0,0},{1,0,0},{0,1,0},3.1,2.2);
        CheckShape("torus inner saddle",Torus(),{4,0,0},{-1,0,0},{0,1,0},2.7,2.1);
        CheckShape("boolean before mapping",Manifold::Sphere(10,48)-Manifold::Cube({6,6,20},true).Translate({8,0,0}),
                   {0,0,10},{0,0,1},{1,0,0},5.2,3.3);
        CheckDepth("flat depth",Manifold::Cube({10,10,2},true),{0.31,0.17,1},{0,0,1},{1,0,0},
                   [](auto p) { return MeshUtils::vec3{0,0,1}; });
        CheckDepth("sphere depth",Manifold::Sphere(10,64),{0,0,10},{0,0,1},{1,0,0},
                   [](auto p) { return linalg::normalize(p); });
        CheckDepth("cylinder depth",Manifold::Cylinder(10,5,5,64),{5,0,5},{1,0,0},{0,1,0},
                   [](auto p) { return linalg::normalize(MeshUtils::vec3{p.x,p.y,0}); });
        auto torusNormal=[](auto p) {
            auto center=7.0*linalg::normalize(MeshUtils::vec3{p.x,p.y,0});
            return linalg::normalize(p-center);
        };
        CheckDepth("torus outer depth",Torus(),{10,0,0},{1,0,0},{0,1,0},torusNormal);
        CheckDepth("torus saddle depth",Torus(),{4,0,0},{-1,0,0},{0,1,0},torusNormal);
        Manifold corner=Manifold::Cube({14,14,14}).Translate({-2,-2,-2})-Manifold::Cube({14,14,14});
        CheckMiter("inner square corner",corner,{0,0,0},{1,1,1},{-1,1,0},{1,1,1});
        CheckMiter("refined inner corner",corner.Refine(5),{0,0,0},{1,1,1},{-1,1,0},{1,1,1});
        CheckMiter("outer square corner",Manifold::Cube({14,14,14}),{0,0,0},{-1,-1,-1},{1,-1,0},{-1,-1,-1});
        Manifold edge=Manifold::Cube({14,14,14}).Translate({-2,-2,-6})-
                       Manifold::Cube({14,14,14}).Translate({0,0,-6});
        CheckMiter("two-face inner edge",edge,{0,0,0},{1,1,0},{-1,1,0},{1,1,0});
        manifold::mat3x4 shear{{1,0,0},{0.4,1,0},{0.2,0.3,1},{-5,4,8}};
        double dy=0.3+std::sqrt(1.09),dx=0.08+0.4*dy+std::sqrt(1.1664);
        CheckMiter("sheared translated corner",corner.Transform(shear),{-5,4,8},
                   {1,1,1},{-1,1,0},{dx,dy,1});
    } catch (const std::exception& exception) {
        std::cerr<<exception.what()<<std::endl;
        return 1;
    }
}
