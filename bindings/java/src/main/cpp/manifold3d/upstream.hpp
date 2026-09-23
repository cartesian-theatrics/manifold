#pragma once
#include <sstream>
#include <stdexcept>

#include "manifold/manifold.h"

namespace clj_manifold {
using ScalarField = double (*)(double, double, double);
inline manifold::Manifold LevelSet(ScalarField field, manifold::vec3 min,
                                   manifold::vec3 max, double edgeLength,
                                   double level, double tolerance,
                                   manifold::ExecutionContext* context) {
  auto sdf = [field](manifold::vec3 p) { return field(p.x, p.y, p.z); };
  // JVM callbacks must execute on the calling thread.
  return context ? context->LevelSet(sdf, {min, max}, edgeLength, level,
                                     tolerance, false)
                 : manifold::Manifold::LevelSet(sdf, {min, max}, edgeLength,
                                                level, tolerance, false);
}
inline manifold::Manifold ReadOBJ(const std::string& text) {
  std::istringstream stream(text);
  return manifold::Manifold::ReadOBJ(stream);
}
inline std::string WriteOBJ(const manifold::Manifold& m) {
  std::ostringstream stream;
  if (!m.WriteOBJ(stream))
    throw std::runtime_error("Manifold OBJ serialization failed");
  return stream.str();
}
}  // namespace clj_manifold
