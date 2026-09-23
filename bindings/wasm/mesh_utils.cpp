// Shared JVM/WASM modeling extensions. Keep algorithms in mesh_utils.hpp;
// this file only converts JS values and translates native exceptions.
#include <emscripten/bind.h>
#include <emscripten/val.h>
#include "model_utils.hpp"
#include "spatial_index.hpp"

namespace js { emscripten::val MeshGL2JS(const manifold::MeshGL&); }

namespace {
using emscripten::val;
using manifold::Manifold;
using manifold::Model;
using MeshUtils::vec3;

template <class F> auto checked(F f) -> decltype(f()) {
  try { return f(); }
  catch (const std::exception& e) {
    val::global("Error").new_(std::string(e.what())).throw_();
    throw;
  }
}

template <class T> std::vector<T> numbers(const val& value) {
  return emscripten::convertJSArrayToNumberVector<T>(value);
}

double number(const val& options, const char* key, double fallback) {
  return options[key].isUndefined() ? fallback : options[key].as<double>();
}

std::vector<double> tuple(const val& options, const char* key,
                          std::vector<double> fallback) {
  if (options[key].isUndefined()) return fallback;
  auto values = numbers<double>(options[key]);
  if (values.size() != fallback.size())
    throw std::invalid_argument(std::string(key) + " has the wrong length");
  return values;
}

size_t property(const Manifold& man, const val& options) {
  double n = number(options, "propIndex", man.NumProp() + 3);
  if (!std::isfinite(n) || n < 3 || n > 2147483645 || n != std::floor(n))
    throw std::invalid_argument("propIndex must be an integer at or after position channels");
  return size_t(n);
}

Model modelTexture(const Model& model,const val& image,const val& options) {
  return checked([&] {
    manifold::ModelTexture p;
    if(!options["mapping"].isUndefined()) p.mapping=options["mapping"].as<std::string>();
    if(!options["name"].isUndefined()) p.name=options["name"].as<std::string>();
    auto o=tuple(options,"origin",{0,0,0}), n=tuple(options,"normal",{0,0,0}),r=tuple(options,"uDirection",{1,0,0});
    p.origin={o[0],o[1],o[2]};p.normal={n[0],n[1],n[2]};p.right={r[0],r[1],r[2]};
    auto size=tuple(options,"size",{1,1}), rect=tuple(options,"uvRect",{0,0,1,1});
    p.width=size[0];p.height=size[1];p.u0=rect[0];p.v0=rect[1];p.u1=rect[2];p.v1=rect[3];
    p.pixelSize=number(options,"pixelSize",0);p.opacity=number(options,"opacity",1);
    auto axes=tuple(options,"axes",{0,2}),scale=tuple(options,"scale",{1,1}),offset=tuple(options,"offset",{0,0});
    p.axisU=int(axes[0]);p.axisV=int(axes[1]);p.scaleU=scale[0];p.scaleV=scale[1];p.offsetU=offset[0];p.offsetV=offset[1];
    p.seamAngle=number(options,"seamAngle",45);p.padding=number(options,"padding",0.01);
    p.pack=options["pack"].isUndefined() || options["pack"].as<bool>();
    p.depthScale=number(options,"depthScale",1);p.depthOffset=number(options,"depthOffset",0);p.depthFade=number(options,"depthFade",-1);
    p.step=!options["depthBoundary"].isUndefined() && options["depthBoundary"].as<std::string>()=="step";
    if(!options["depthValues"].isUndefined()) {
      auto values=numbers<double>(options["depthValues"]);
      p.SetDepth(values.data(),values.size(),int(number(options,"depthWidth",0)),int(number(options,"depthHeight",0)));
    }
    if(!options["depthImage"].isUndefined()) {
      auto bytes=numbers<uint8_t>(options["depthImage"]);
      p.SetDepthImage(bytes.data(),bytes.size());
    }
    auto bytes=numbers<uint8_t>(image);
    return manifold::ModelIO::Texture(model,bytes.data(),bytes.size(),p);
  });
}

Manifold planar(const Manifold& man, const val& options) {
  return checked([&] {
    auto axes = tuple(options, "axes", {0, 2});
    auto scale = tuple(options, "scale", {1, 1});
    auto offset = tuple(options, "offset", {0, 0});
    if (axes[0] != std::floor(axes[0]) || axes[1] != std::floor(axes[1]))
      throw std::invalid_argument("UV axes must be integers");
    return MeshUtils::ApplyPlanarUV(man, property(man, options), int(axes[0]),
                                    int(axes[1]), scale[0], scale[1], offset[0], offset[1]);
  });
}

Manifold unwrap(const Manifold& man, const val& options) {
  return checked([&] {
    bool pack = options["pack"].isUndefined() || options["pack"].as<bool>();
    return MeshUtils::UnwrapUV(man, property(man, options),
        number(options, "seamAngle", 45), number(options, "scale", 1),
        number(options, "padding", 0.01), pack);
  });
}

Manifold geodesic(const Manifold& man, const val& options) {
  return checked([&] {
    auto origin = tuple(options, "origin", {0, 0, 0});
    auto normal = tuple(options, "normal", {0, 0, 0});
    auto right = tuple(options, "uDirection", {1, 0, 0});
    auto size = tuple(options, "size", {1, 1});
    auto rect = tuple(options, "uvRect", {0, 0, 1, 1});
    auto outside = tuple(options, "outsideUV", {0, 0});
    double pixel = number(options, "pixelSize", std::min(size[0], size[1]) / 32);
    bool step = !options["depthBoundary"].isUndefined() &&
                options["depthBoundary"].as<std::string>() == "step";
    double fade = number(options, "depthFade", step ? 0 :
                         std::min(2 * pixel, std::min(size[0], size[1]) / 2));
    double scale = number(options, "depthScale", 1);
    double offset = number(options, "depthOffset", 0);
    MeshUtils::SurfaceUV::DepthField depth;
    if (!options["depthImage"].isUndefined()) {
      auto bytes = numbers<unsigned char>(options["depthImage"]);
      if (bytes.size() > size_t(std::numeric_limits<int>::max()))
        throw std::invalid_argument("Depth image exceeds decoder limits");
      int w = 0, h = 0, channels = 0;
      if (!stbi_info_from_memory(bytes.data(), bytes.size(), &w, &h, &channels))
        throw std::invalid_argument("Cannot decode depth image");
      if (w < 2 || h < 2 || uint64_t(w) * h > 2000000)
        throw std::invalid_argument("Depth image must be at least 2x2 and at most two million pixels");
      std::unique_ptr<stbi_us, decltype(&stbi_image_free)> image(
          stbi_load_16_from_memory(bytes.data(), bytes.size(), &w, &h, &channels, 1), stbi_image_free);
      if (!image) throw std::invalid_argument("Cannot decode depth image");
      std::vector<double> values(size_t(w) * h);
      for (size_t i = 0; i < values.size(); ++i) values[i] = image.get()[i] / 65535.0;
      depth = {std::move(values), w, h, scale, offset, fade, step};
    } else if (!options["depthValues"].isUndefined()) {
      depth = {numbers<double>(options["depthValues"]),
               options["depthWidth"].as<int>(), options["depthHeight"].as<int>(),
               scale, offset, fade, step};
    }
    return MeshUtils::GeodesicUV(man, property(man, options),
        origin[0], origin[1], origin[2], normal[0], normal[1], normal[2],
        right[0], right[1], right[2], size[0], size[1],
        rect[0], rect[1], rect[2] - rect[0], rect[3] - rect[1],
        outside[0], outside[1], pixel, depth);
  });
}

Manifold color(const Manifold& man, const val& rgba, size_t propIndex) {
  return checked([&] {
    auto c = numbers<double>(rgba);
    if (c.size() != 4) throw std::invalid_argument("Color requires four components");
    if (propIndex < 3 || propIndex > 1024)
      throw std::invalid_argument("Color property index must be between 3 and 1024");
    // Native property updates preserve topology, seams, and original face IDs.
    const int oldCount = man.NumProp();
    const int offset = static_cast<int>(propIndex) - 3;
    const int count = std::max(oldCount, offset + 4);
    return man.SetProperties(count, [=](double* out, vec3, const double* old) {
      std::fill(out, out + count, 0.0);
      std::copy(old, old + oldCount, out);
      std::copy(c.begin(), c.end(), out + offset);
    });
  });
}

Manifold polyhedron(const val& vertices, const val& faces) {
  return checked([&] {
    std::vector<vec3> v;
    std::vector<std::vector<uint32_t>> f;
    for (unsigned i = 0; i < vertices["length"].as<unsigned>(); ++i) {
      auto p = numbers<double>(vertices[i]);
      if (p.size() != 3) throw std::invalid_argument("Vertices require three coordinates");
      v.push_back({p[0], p[1], p[2]});
    }
    for (unsigned i = 0; i < faces["length"].as<unsigned>(); ++i) {
      auto face = numbers<uint32_t>(faces[i]);
      if (face.size() < 3) throw std::invalid_argument("Faces require at least three vertices");
      for (auto index : face) if (index >= v.size())
        throw std::invalid_argument("Face index is out of bounds");
      f.push_back(std::move(face));
    }
    return MeshUtils::Polyhedron(v, f);
  });
}

Manifold surface(const val& rows, double pixelWidth) {
  return checked([&] {
    unsigned height = rows["length"].as<unsigned>();
    if (height < 2) throw std::invalid_argument("Surface requires at least two rows");
    unsigned width = rows[0]["length"].as<unsigned>();
    if (width < 2 || uint64_t(width) * height > 2000000)
      throw std::invalid_argument("Invalid surface dimensions");
    std::vector<float> values;
    for (unsigned y = 0; y < height; ++y) {
      auto row = numbers<float>(rows[y]);
      if (row.size() != width) throw std::invalid_argument("Surface rows must have equal length");
      values.insert(values.end(), row.begin(), row.end());
    }
    // Match core/surface's existing JVM heatmap layout (outer count is width).
    return MeshUtils::CreateSurface(values.data(), 1, height, width, pixelWidth);
  });
}

Manifold loft(const val& sections, const val& frames, const std::string& algorithm) {
  return checked([&] {
    unsigned count = frames["length"].as<unsigned>();
    if (count < 2 || count != sections["length"].as<unsigned>())
      throw std::invalid_argument("Loft requires at least two matching sections and frames");
    std::vector<manifold::Polygons> polygons;
    std::vector<MeshUtils::mat3x4> transforms;
    for (unsigned i = 0; i < count; ++i) {
      polygons.push_back(sections[i].as<manifold::CrossSection>().ToPolygons());
      auto f = numbers<double>(frames[i]);
      if (f.size() != 12) throw std::invalid_argument("Loft frames require 12 column-major entries");
      transforms.push_back({{f[0], f[1], f[2]}, {f[3], f[4], f[5]},
                            {f[6], f[7], f[8]}, {f[9], f[10], f[11]}});
    }
    if (algorithm != "isomorphic" && algorithm != "eager-nearest-neighbor")
      throw std::invalid_argument("Unknown loft algorithm");
    return MeshUtils::Loft(polygons, transforms, algorithm == "isomorphic"
        ? MeshUtils::LoftAlgorithm::Isomorphic : MeshUtils::LoftAlgorithm::EagerNearestNeighbor);
  });
}

manifold::CrossSection text(const std::string& font, const std::string& content,
                           unsigned height, int resolution, int fill) {
  return checked([&] { return manifold::CrossSection::Text(font, content, height,
      resolution, static_cast<manifold::CrossSection::FillRule>(fill)); });
}
Manifold loadSurface(const std::string& path, double pixelWidth) {
  return checked([&] { return MeshUtils::CreateSurface(path, pixelWidth); });
}
Manifold loadImage(const std::string& path, float depth, double pixelWidth) {
  return checked([&] { return MeshUtils::LoadImage(path, depth, pixelWidth); });
}
Manifold plySurface(const std::string& path, double cellSize, double offset, double scale) {
  return checked([&] { return MeshUtils::PlyToSurface(path, cellSize, offset, scale); });
}
} // namespace

EMSCRIPTEN_BINDINGS(clj_manifold_extensions) {
  using namespace emscripten;
  using MeshUtils::SpatialIndex;
  function("spatialIndex", optional_override([](const Manifold& m) {
    return checked([&] { return SpatialIndex(m); });
  }));
  class_<SpatialIndex>("SpatialIndex")
    .function("rayCast", optional_override([](const SpatialIndex& s,double x,double y,double z,double dx,double dy,double dz,double limit) {
      return checked([&] { auto v=s.RayCast(x,y,z,dx,dy,dz,limit);
        auto a=val::array(); for(double n:v) a.call<void>("push",n); return a; });
    }))
    .function("closestPoint", optional_override([](const SpatialIndex& s,double x,double y,double z) {
      return checked([&] { auto v=s.ClosestPoint(x,y,z);
        auto a=val::array(); for(double n:v) a.call<void>("push",n); return a; });
    }))
    .function("classifyPoint", optional_override([](const SpatialIndex& s,double x,double y,double z,double tolerance) {
      return checked([&] { return s.ClassifyPoint(x,y,z,tolerance); });
    }))
    .function("overlaps", optional_override([](const SpatialIndex& a,const SpatialIndex& b,double tolerance) {
      return checked([&] { return a.Overlaps(b,tolerance); });
    }));
  function("modelTexture", &modelTexture);
  function("modelGLB", optional_override([](const Model& m,int tile) {
    return checked([&] { auto bytes=manifold::ModelIO::ExportGLB(m,tile);
      return val(typed_memory_view(bytes.size(),bytes.data())).call<val>("slice"); });
  }));
  class_<Model>("Model")
    .constructor<const Manifold&>()
    .function("geometry", &Model::Geometry)
    .function("getMesh", optional_override([](const Model& m) { return js::MeshGL2JS(m.GetMeshGL()); }))
    .function("getMesh", optional_override([](const Model& m,int i) { return js::MeshGL2JS(m.GetMeshGL(i)); }))
    .function("translate", optional_override([](const Model& m,const val& v) { return checked([&] {auto a=numbers<double>(v); if(a.size()!=3) throw std::invalid_argument("Expected vec3"); return m.Translate({a[0],a[1],a[2]});}); }))
    .function("scale", optional_override([](const Model& m,const val& v) { return checked([&] {auto a=numbers<double>(v); if(a.size()!=3) throw std::invalid_argument("Expected vec3"); return m.Scale({a[0],a[1],a[2]});}); }))
    .function("mirror", optional_override([](const Model& m,const val& v) { return checked([&] {auto a=numbers<double>(v); if(a.size()!=3) throw std::invalid_argument("Expected vec3"); return m.Mirror({a[0],a[1],a[2]});}); }))
    .function("rotate", optional_override([](const Model& m,const val& v) { return checked([&] {auto a=numbers<double>(v); if(a.size()!=3) throw std::invalid_argument("Expected vec3"); return m.Rotate(a[0],a[1],a[2]);}); }))
    .function("transform", optional_override([](const Model& m,const val& v) { return checked([&] {
      auto a=numbers<double>(v); if(a.size()!=16) throw std::invalid_argument("Expected column-major mat4");
      return m.Transform({{a[0],a[1],a[2]},{a[4],a[5],a[6]},{a[8],a[9],a[10]},{a[12],a[13],a[14]}}); }); }))
    .function("booleanOp", optional_override([](const Model& a,const Model& b,int op) { return checked([&] {if(op<0 || op>2) throw std::invalid_argument("Invalid boolean operation");return a.Boolean(b,manifold::OpType(op));}); }))
    .function("compose", optional_override([](const Model& a,const Model& b) { return checked([&] {return a.Compose(b);}); }))
    .function("color", optional_override([](const Model& m,const val& v) { return checked([&] {auto a=numbers<double>(v); if(a.size()!=4) throw std::invalid_argument("Expected RGBA"); return m.Color({a[0],a[1],a[2],a[3]});}); }))
    .function("refine", optional_override([](const Model& m,int n) {return checked([&]{return m.Refine(n);});}))
    .function("refineToLength", optional_override([](const Model& m,double n) {return checked([&]{return m.RefineToLength(n);});}))
    .function("smoothOut", optional_override([](const Model& m,double a,double s) {return checked([&]{return m.SmoothOut(a,s);});}))
    .function("calculateNormals", optional_override([](const Model& m,int i,double a) {return checked([&]{return m.CalculateNormals(i,a);});}))
    .function("decompose", optional_override([](const Model& m) { return checked([&] {auto a=val::array(); for(const auto& part:m.Decompose()) a.call<void>("push",val(part)); return a;}); }))
    .function("boundingBox", optional_override([](const Model& m) {auto box=m.BoundingBox();auto v=val::object();auto low=val::array(),high=val::array(); for(int i=0;i<3;++i){low.call<void>("push",box.min[i]);high.call<void>("push",box.max[i]);}v.set("min",low);v.set("max",high);return v;}))
    .function("sampleColor", optional_override([](const Model& m,unsigned f,double u,double v) {return checked([&] {auto c=m.SampleColor(f,u,v);auto a=val::array();for(int i=0;i<4;++i)a.call<void>("push",c[i]);return a;});}))
    .function("status", &Model::Status).function("isEmpty", &Model::IsEmpty)
    .function("numVert", &Model::NumVert).function("numTri", &Model::NumTri)
    .function("numProp", &Model::NumProp).function("numEdge", &Model::NumEdge)
    .function("layerCount", &Model::LayerCount).function("imageCount", &Model::ImageCount)
    .function("surfaceCount", &Model::SurfaceCount).function("genus", &Model::Genus)
    .function("volume", &Model::Volume).function("surfaceArea", &Model::SurfaceArea);
  emscripten::function("applyPlanarUV", &planar);
  emscripten::function("unwrapUV", &unwrap);
  emscripten::function("geodesicUV", &geodesic);
  emscripten::function("colorVertices", &color);
  emscripten::function("polyhedron", &polyhedron);
  emscripten::function("surface", &surface);
  emscripten::function("loft", &loft);
  emscripten::function("text", &text);
  emscripten::function("loadSurface", &loadSurface);
  emscripten::function("loadImage", &loadImage);
  emscripten::function("plyToSurface", &plySurface);
}
