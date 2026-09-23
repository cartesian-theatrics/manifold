# Clojure binding integration: Manifold 3.5.3

The fork is merged with upstream tag `v3.5.3`, commit
`0edd9d54876f3135e431575214dd6d8a72866fee` (2026-09-07).
The stable release retains the fill-rule, tolerance and smoothing interfaces
used by existing Clojure models; development after that tag removes several of
those interfaces. This integration does not claim compatibility with master.

## Retained fork features

- JavaCPP JNI bindings and the Assimp mesh import/export interface.
- Native Model materials, textures, color/normal/UV handling and surface mapping.
- MeshUtils lofting, PLY loading, image/depth mapping and spatial index helpers.
- CrossSection text, slices and topology inspection (`GetVertices`,
  `GetHalfedges`, `GetTriangles`, `GetFaceNormals`). Topology inspection now uses
  upstream's halfedge structure-of-arrays; triangle indices address geometric
  vertices, independent of property seams.
- WASM Model/MeshUtils bindings, CommonJS/ESM selection, virtual filesystem,
  callback ownership fixes and exception support.

## Added binding coverage

Both front ends expose Minkowski sum/difference, solid simplification, native
segment ray hits, execution contexts, progress/cancellation, bevel offsets and
OBJ string I/O. Previously unexposed tolerance/refinement, curvature, normal
smoothing, minimum gap and SDF level-set APIs are also available in CLJ and CLJS.
MeshGL run flags and tolerance survive a portable data round-trip. See
`../clj-manifold3d/README.md` for function names, defaults and context limitations.

The bridge `bindings/java/src/main/cpp/manifold3d/upstream.hpp` adapts stream I/O
and scalar-field callbacks. JVM SDF callbacks run serially on the calling thread.
The JS RayHit bridge represents face IDs as numbers (the WASM mesh limits are
below JS's exact integer limit). Context/OBJ JNI entry points initialize the
packaged native library even when they are the first entry point in a process.
Assimp import/export errors throw in Release builds rather than relying on
compiled-out debug assertions.

## Build and validate

On Linux, install a C++17 compiler, CMake, Maven, a full JDK, and matching Assimp
headers/runtime. FetchContent downloads pinned Clipper2, TextToPolygon, FreeType
and test dependencies. Keep native and WASM build directories separate.

```sh
cmake -S . -B build-upstream -DCMAKE_BUILD_TYPE=Release \
  -DMANIFOLD_TEST=ON -DMANIFOLD_EXPORT=ON -DMANIFOLD_PAR=OFF \
  -DMANIFOLD_PYBIND=OFF -DMANIFOLD_JSBIND=OFF
cmake --build build-upstream --parallel 4
ctest --test-dir build-upstream --output-on-failure
mvn -f bindings/java/pom.xml -Dmanifold.build.dir="$PWD/build-upstream" package
```

`MANIFOLD_EXPORT` restores the fork's JNI file I/O symbols (upstream moved that
code into extras). The POM stages `libmanifold.so` from the selected build,
following its versioned symlink. Its historical `1.0.39` local jar filename is
retained because existing `deps.edn` files reference it; no release has been
published under that version by this update. Only that compatibility jar remains
versioned under `target`; Maven intermediate files are ignored. Restart JVMs that loaded an older
jar before using the rebuilt one.

With Emscripten activated, build and test from the sibling wrapper repository:

```sh
cd ../clj-manifold3d
npm ci
npm run build:wasm
npm test
clojure -M:clj-dev:test-regressions
```

Validated on Linux x86-64 (serial native build) and Emscripten 3.1.64 in Node
and Chromium, including optimized CLJS. macOS/Windows JNI packaging and native
parallel builds have not been validated as part of this update. The Linux JNI
workflow builds/tests artifacts without publishing them.
