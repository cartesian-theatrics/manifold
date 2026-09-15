#pragma once

#include <array>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <queue>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include "manifold/linalg.h"
#include "manifold/polygon.h"
#include "manifold/manifold.h"
#include "manifold/cross_section.h"
#include "happly.h"
#include "buffer_utils.hpp"
#include "matrix_transforms.hpp"
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

namespace MeshUtils {

using vec2   = linalg::vec<double, 2>;
using vec3   = linalg::vec<double, 3>;
using vec4   = linalg::vec<double, 4>;
using ivec3  = linalg::vec<int, 3>;
using mat2x2 = linalg::mat<double, 2, 2>;
using mat3x3 = linalg::mat<double, 3, 3>;
using mat4x3 = linalg::mat<double, 4, 3>;
using mat3x4 = linalg::mat<double, 3, 4>;

manifold::Manifold CreateManifold(const std::vector<vec3>& vertices, const std::vector<ivec3> triVerts) {
    manifold::MeshGL mesh;
    mesh.triVerts.reserve(3 * triVerts.size());
    for (auto& triVert: triVerts) {
        mesh.triVerts.push_back(triVert[0]);
        mesh.triVerts.push_back(triVert[1]);
        mesh.triVerts.push_back(triVert[2]);
    }
    mesh.vertProperties.reserve(3 * vertices.size());
    for (auto& vert: vertices) {
        mesh.vertProperties.push_back(vert[0]);
        mesh.vertProperties.push_back(vert[1]);
        mesh.vertProperties.push_back(vert[2]);
    }

    return manifold::Manifold(mesh);
}

manifold::Manifold CreateSurface(const float* vertProperties, int numProps, int width, int height, float pixelWidth = 1.0) {
    // Create the MeshGL structure
    manifold::MeshGL meshGL;

    // Set number of vertex properties based on numProps
    meshGL.numProp = numProps + 2;
    int numVerts = width * height * meshGL.numProp * 2;
    meshGL.vertProperties.reserve(numVerts);
    int numTopBottomTriangles = 4 * (width - 1) * (height - 1);
    int numEdgeTriangles = 4 * (height - 1) + 4 * (width - 1);
    int numTriangles = numTopBottomTriangles + numEdgeTriangles;
    meshGL.triVerts.reserve(3 * numTriangles);

    // Generate top surface vertices and properties
    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            const float* props = &vertProperties[(i * width + j) * numProps];
            float x = j * pixelWidth;
            float y = i * pixelWidth;
            float z = props[0];  // Height (z)

            // Add vertex properties (x, y, z)
            meshGL.vertProperties.push_back(x);
            meshGL.vertProperties.push_back(y);
            meshGL.vertProperties.push_back(z);

            // Add additional properties from index 3 to numProps
            for (int k = 1; k < numProps; ++k) {
                meshGL.vertProperties.push_back(props[k]);
            }
        }
    }

    // Generate bottom surface vertices (z = 0)
    int bottomOffset = width * height;  // Bottom vertices start after the top vertices
    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            // Add bottom vertex properties (x, y, z=0)
            meshGL.vertProperties.push_back(j * pixelWidth);
            meshGL.vertProperties.push_back(i * pixelWidth);
            meshGL.vertProperties.push_back(0.0);

            // Set remaining properties to 0 from index 3 to numProps
            for (int k = 1; k < numProps; ++k) {
                meshGL.vertProperties.push_back(0.0);
            }
        }
    }

    // Step 3: Generate triangles for the top and bottom surfaces
    for (int i = 0; i < height - 1; ++i) {
        for (int j = 0; j < width - 1; ++j) {
            int topLeft = i * width + j;
            int topRight = i * width + (j + 1);
            int bottomLeft = (i + 1) * width + j;
            int bottomRight = (i + 1) * width + (j + 1);

            // Top surface triangles (counterclockwise)
            meshGL.triVerts.push_back(bottomRight);
            meshGL.triVerts.push_back(bottomLeft);
            meshGL.triVerts.push_back(topLeft);

            meshGL.triVerts.push_back(topRight);
            meshGL.triVerts.push_back(bottomRight);
            meshGL.triVerts.push_back(topLeft);

            // Bottom surface triangles (clockwise)
            int bTopLeft = bottomOffset + topLeft;
            int bTopRight = bottomOffset + topRight;
            int bBottomLeft = bottomOffset + bottomLeft;
            int bBottomRight = bottomOffset + bottomRight;
            meshGL.triVerts.push_back(bBottomLeft);
            meshGL.triVerts.push_back(bBottomRight);
            meshGL.triVerts.push_back(bTopLeft);

            meshGL.triVerts.push_back(bTopLeft);
            meshGL.triVerts.push_back(bBottomRight);
            meshGL.triVerts.push_back(bTopRight);
        }
    }

    // Step 4: Generate triangles for the sides (left, right, top, bottom)
    // Left edge
    for (int i = 0; i < height - 1; ++i) {
        int tTop = i * width;
        int tBottom = (i + 1) * width;
        int bTop = bottomOffset + tTop;
        int bBottom = bottomOffset + tBottom;

        meshGL.triVerts.push_back(tTop);
        meshGL.triVerts.push_back(tBottom);
        meshGL.triVerts.push_back(bBottom);

        meshGL.triVerts.push_back(tTop);
        meshGL.triVerts.push_back(bBottom);
        meshGL.triVerts.push_back(bTop);
    }

    // Right edge
    for (int i = 0; i < height - 1; ++i) {
        int tTop = i * width + (width - 1);
        int tBottom = (i + 1) * width + (width - 1);
        int bTop = bottomOffset + tTop;
        int bBottom = bottomOffset + tBottom;

        meshGL.triVerts.push_back(tTop);
        meshGL.triVerts.push_back(bBottom);
        meshGL.triVerts.push_back(tBottom);

        meshGL.triVerts.push_back(tTop);
        meshGL.triVerts.push_back(bTop);
        meshGL.triVerts.push_back(bBottom);
    }

    // Top edge
    for (int j = 0; j < width - 1; ++j) {
        int tLeft = j;
        int tRight = j + 1;
        int bLeft = bottomOffset + tLeft;
        int bRight = bottomOffset + tRight;

        meshGL.triVerts.push_back(bLeft);
        meshGL.triVerts.push_back(bRight);
        meshGL.triVerts.push_back(tRight);

        meshGL.triVerts.push_back(tLeft);
        meshGL.triVerts.push_back(bLeft);
        meshGL.triVerts.push_back(tRight);
    }

    // Bottom edge
    for (int j = 0; j < width - 1; ++j) {
        int tLeft = (height - 1) * width + j;
        int tRight = (height - 1) * width + j + 1;
        int bLeft = bottomOffset + tLeft;
        int bRight = bottomOffset + tRight;

        meshGL.triVerts.push_back(tLeft);
        meshGL.triVerts.push_back(tRight);
        meshGL.triVerts.push_back(bRight);

        meshGL.triVerts.push_back(tLeft);
        meshGL.triVerts.push_back(bRight);
        meshGL.triVerts.push_back(bLeft);
    }

    // Create and validate the manifold
    manifold::Manifold solid = manifold::Manifold(meshGL);
    manifold::Manifold::Error status = solid.Status();
    if (status != manifold::Manifold::Error::NoError) {
        throw std::runtime_error("Generated manifold is invalid.");
    }

    return solid;
}


manifold::Manifold PlyToSurface(const std::string &filepath, double cellSize, double zOffset, double scaleFactor) {
    // Create a reader for the PLY file
    happly::PLYData plyIn(filepath);

    std::vector<float> vX = plyIn.getElement("vertex").getProperty<float>("x");
    std::vector<float> vY = plyIn.getElement("vertex").getProperty<float>("y");
    std::vector<float> vZ = plyIn.getElement("vertex").getProperty<float>("z");

    std::vector<uint8_t> vR = plyIn.getElement("vertex").getProperty<uint8_t>("red");
    std::vector<uint8_t> vG = plyIn.getElement("vertex").getProperty<uint8_t>("green");
    std::vector<uint8_t> vB = plyIn.getElement("vertex").getProperty<uint8_t>("blue");

    float min_x = *std::min_element(vX.begin(), vX.end());
    float max_x = *std::max_element(vX.begin(), vX.end());
    float min_y = *std::min_element(vY.begin(), vY.end());
    float max_y = *std::max_element(vY.begin(), vY.end());
    float min_z = *std::min_element(vZ.begin(), vZ.end());

    // Calculate the spans for x and y
    double x_span = (max_x - min_x) * scaleFactor;
    double y_span = (max_y - min_y) * scaleFactor;

    int grid_resolution_x = static_cast<int>(x_span / cellSize);
    int grid_resolution_y = static_cast<int>(y_span / cellSize);

    // Ensure at least one cell is created in both directions
    grid_resolution_x = std::max(1, grid_resolution_x);
    grid_resolution_y = std::max(1, grid_resolution_y);

    // Initialize grid structures for z-value sums, color sums, and point counts
    std::vector<std::vector<float>> z_sum(grid_resolution_x, std::vector<float>(grid_resolution_y, 0.0));
    std::vector<std::vector<float>> r_sum(grid_resolution_x, std::vector<float>(grid_resolution_y, 0.0));
    std::vector<std::vector<float>> g_sum(grid_resolution_x, std::vector<float>(grid_resolution_y, 0.0));
    std::vector<std::vector<float>> b_sum(grid_resolution_x, std::vector<float>(grid_resolution_y, 0.0));
    std::vector<std::vector<int>> point_count(grid_resolution_x, std::vector<int>(grid_resolution_y, 0));

    // Process each point in the PLY file
    for (size_t i = 0; i < vX.size(); ++i) {
        float x = vX[i];
        float y = vY[i];
        float z = vZ[i];
        float r = static_cast<float>(vR[i]) / 255.0;
        float g = static_cast<float>(vG[i]) / 255.0;
        float b = static_cast<float>(vB[i]) / 255.0;

        // Find the corresponding grid cell indices
        int grid_x = static_cast<int>(((x - min_x) * scaleFactor) / cellSize);
        int grid_y = static_cast<int>(((y - min_y) * scaleFactor) / cellSize);

        // Ensure the point falls within the grid bounds
        if (grid_x >= 0 && grid_x < grid_resolution_x && grid_y >= 0 && grid_y < grid_resolution_y) {
            // Accumulate the z and color values in the corresponding grid cell
            z_sum[grid_x][grid_y] += (z - min_z) * scaleFactor;
            r_sum[grid_x][grid_y] += r;
            g_sum[grid_x][grid_y] += g;
            b_sum[grid_x][grid_y] += b;
            point_count[grid_x][grid_y] += 1;
        }
    }
    int nProp = 4;

    // Initialize vertProperties to store the flattened vertex data
    std::vector<float> vertProperties;
    vertProperties.reserve(grid_resolution_x * grid_resolution_y * nProp);  // Reserve space for z, r, g, b per cell

    // Compute the average height and color for each grid cell and populate vertProperties
    for (int i = 0; i < grid_resolution_x; ++i) {
        for (int j = 0; j < grid_resolution_y; ++j) {
            if (point_count[i][j] > 0) {
                // Calculate the average height, apply offset, and set default color
                float avg_z = (z_sum[i][j] / point_count[i][j]) + zOffset;
                float avg_r = r_sum[i][j] / point_count[i][j];
                float avg_g = g_sum[i][j] / point_count[i][j];
                float avg_b = b_sum[i][j] / point_count[i][j];
                // Push x, y, z, r, g, b for each grid cell in row-major order
                float* props = &vertProperties[(j * grid_resolution_x + i) * nProp];
                props[0] = avg_z;
                props[1] = avg_r;
                props[2] = avg_g;
                props[3] = avg_b;
            } else {
                float* props = &vertProperties[(j * grid_resolution_x + i) * nProp];
                props[0] = 10.0;
                props[1] = 0.0;
                props[2] = 0.0;
                props[3] = 0.0;
            }

        }
    }

    // Pass vertProperties to CreateSurface as a pointer and specify numProps = 6
    return CreateSurface(vertProperties.data(), nProp, grid_resolution_x, grid_resolution_y, cellSize);
}

manifold::Manifold ColorVertices(const manifold::Manifold& man, const vec4 color, size_t propIndex = 3) {
    manifold::MeshGL mesh = man.GetMeshGL();
    const std::vector<float>& vertProps = mesh.vertProperties;
    size_t numProps = mesh.numProp;

    size_t numNewProps = std::max(propIndex + 4, static_cast<size_t>(numProps));
    std::vector<float> newVertProps;
    newVertProps.resize(numNewProps * mesh.NumVert());

    for (size_t i = 0; i < mesh.NumVert(); ++i) {
        for (size_t j = 0; j < numProps; ++j) {
            newVertProps[i * numNewProps + j] = vertProps[i * numProps + j];
        }

        for (size_t j = 0; j < 4; ++j) {
            newVertProps[i * numNewProps + propIndex + j] = color[j];
        }
    }

    manifold::MeshGL newMesh;
    newMesh.vertProperties = std::move(newVertProps);
    newMesh.numProp = numNewProps;
    newMesh.triVerts = mesh.triVerts;

    return manifold::Manifold(newMesh);
}

/**
 * Add planar UV coordinates to a Manifold without crossing the Java/C++
 * boundary once per vertex. The axes are position channels in [0, 1, 2], and
 * the generated coordinates are:
 *
 *   u = position[axisU] * scaleU + offsetU
 *   v = position[axisV] * scaleV + offsetV
 *
 * The property index is absolute in MeshGL, so 3 is the first channel after
 * position. Existing MeshGL metadata is retained so property seams and input
 * relations remain available to the reconstructed Manifold.
 */
manifold::Manifold ApplyPlanarUV(const manifold::Manifold& man,
                                 size_t propIndex,
                                 int axisU,
                                 int axisV,
                                 double scaleU,
                                 double scaleV,
                                 double offsetU,
                                 double offsetV) {
    if (propIndex < 3) {
        throw std::invalid_argument("UV property index must be at least 3");
    }
    if (axisU < 0 || axisU > 2 || axisV < 0 || axisV > 2 || axisU == axisV) {
        throw std::invalid_argument("UV axes must be distinct position axes in [0, 2]");
    }
    if (!std::isfinite(scaleU) || !std::isfinite(scaleV) ||
        !std::isfinite(offsetU) || !std::isfinite(offsetV)) {
        throw std::invalid_argument("UV scale and offset values must be finite");
    }

    manifold::MeshGL mesh = man.GetMeshGL();
    const size_t oldNumProps = mesh.numProp;
    const size_t newNumProps = std::max(propIndex + 2, oldNumProps);
    std::vector<float> newVertProps(newNumProps * mesh.NumVert(), 0.0f);

    for (size_t vertex = 0; vertex < mesh.NumVert(); ++vertex) {
        const size_t oldOffset = vertex * oldNumProps;
        const size_t newOffset = vertex * newNumProps;
        for (size_t property = 0; property < oldNumProps; ++property) {
            newVertProps[newOffset + property] =
                mesh.vertProperties[oldOffset + property];
        }
        newVertProps[newOffset + propIndex] = static_cast<float>(
            mesh.vertProperties[oldOffset + axisU] * scaleU + offsetU);
        newVertProps[newOffset + propIndex + 1] = static_cast<float>(
            mesh.vertProperties[oldOffset + axisV] * scaleV + offsetV);
    }

    mesh.numProp = newNumProps;
    mesh.vertProperties = std::move(newVertProps);
    return manifold::Manifold(mesh);
}

struct UVVec2 {
    double x = 0.0;
    double y = 0.0;
};

struct UVVec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

UVVec3 UVSub(const UVVec3& a, const UVVec3& b) {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}

double UVDot(const UVVec3& a, const UVVec3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

UVVec3 UVCross(const UVVec3& a, const UVVec3& b) {
    return {a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x};
}

double UVLength(const UVVec3& v) {
    return std::sqrt(UVDot(v, v));
}

struct UVUnionFind {
    std::vector<int> parent;

    explicit UVUnionFind(size_t size) : parent(size) {
        for (size_t i = 0; i < size; ++i) parent[i] = static_cast<int>(i);
    }

    int find(int value) {
        int root = value;
        while (parent[root] != root) root = parent[root];
        while (parent[value] != value) {
            int next = parent[value];
            parent[value] = root;
            value = next;
        }
        return root;
    }

    void unite(int a, int b) {
        a = find(a);
        b = find(b);
        if (a != b) parent[b] = a;
    }
};

struct UVEdgeRecord {
    int a = -1;
    int b = -1;
    int face0 = -1;
    int edge0 = -1;
    int face1 = -1;
    int edge1 = -1;
    bool seam = false;
};

struct UVEquation {
    std::array<int, 6> indexes{};
    std::array<double, 6> coefficients{};
    int count = 0;
    double rhs = 0.0;

    void add(int index, double coefficient) {
        if (index < 0 || std::abs(coefficient) < 1e-15) return;
        indexes[count] = index;
        coefficients[count] = coefficient;
        ++count;
    }
};

struct UVChartData {
    std::vector<int> faces;
    std::vector<int> vertices;
    std::unordered_map<int, int> local;
    std::vector<UVVec2> uv;
};

std::vector<double> SolveUVEquations(const std::vector<UVEquation>& equations,
                                     int numVariables) {
    std::vector<double> solution(numVariables, 0.0);
    if (numVariables == 0 || equations.empty()) return solution;

    std::vector<double> rhs(numVariables, 0.0);
    std::vector<double> diagonal(numVariables, 0.0);
    for (const UVEquation& equation : equations) {
        for (int i = 0; i < equation.count; ++i) {
            const int rowIndex = equation.indexes[i];
            const double rowCoefficient = equation.coefficients[i];
            rhs[rowIndex] += rowCoefficient * equation.rhs;
            diagonal[rowIndex] += rowCoefficient * rowCoefficient;
        }
    }

    double maxDiagonal = 0.0;
    for (double value : diagonal) maxDiagonal = std::max(maxDiagonal, value);
    const double regularization = std::max(1e-14, maxDiagonal * 1e-10);
    for (double& value : diagonal) value += regularization;

    auto multiply = [&equations, regularization, numVariables](
                        const std::vector<double>& input) {
        std::vector<double> output(numVariables, 0.0);
        for (const UVEquation& equation : equations) {
            double rowValue = 0.0;
            for (int i = 0; i < equation.count; ++i) {
                rowValue += equation.coefficients[i] * input[equation.indexes[i]];
            }
            for (int i = 0; i < equation.count; ++i) {
                output[equation.indexes[i]] +=
                    equation.coefficients[i] * rowValue;
            }
        }
        for (int i = 0; i < numVariables; ++i) {
            output[i] += regularization * input[i];
        }
        return output;
    };

    auto dot = [](const std::vector<double>& a,
                  const std::vector<double>& b) {
        double result = 0.0;
        for (size_t i = 0; i < a.size(); ++i) result += a[i] * b[i];
        return result;
    };

    std::vector<double> residual = rhs;
    std::vector<double> preconditioned(numVariables, 0.0);
    std::vector<double> direction(numVariables, 0.0);
    for (int i = 0; i < numVariables; ++i) {
        preconditioned[i] = residual[i] / diagonal[i];
        direction[i] = preconditioned[i];
    }

    double residualDot = dot(residual, preconditioned);
    const double rhsLength = std::sqrt(std::max(0.0, dot(rhs, rhs)));
    if (rhsLength < 1e-14) return solution;
    const double targetResidual = std::max(1e-12, rhsLength * 1e-8);
    const int maxIterations = std::min(2000, std::max(100, numVariables * 2));

    for (int iteration = 0; iteration < maxIterations; ++iteration) {
        std::vector<double> matrixDirection = multiply(direction);
        const double denominator = dot(direction, matrixDirection);
        if (std::abs(denominator) < 1e-30) break;
        const double alpha = residualDot / denominator;
        for (int i = 0; i < numVariables; ++i) {
            solution[i] += alpha * direction[i];
            residual[i] -= alpha * matrixDirection[i];
        }

        const double residualLength =
            std::sqrt(std::max(0.0, dot(residual, residual)));
        if (residualLength <= targetResidual) break;

        for (int i = 0; i < numVariables; ++i) {
            preconditioned[i] = residual[i] / diagonal[i];
        }
        const double nextResidualDot = dot(residual, preconditioned);
        if (std::abs(residualDot) < 1e-30) break;
        const double beta = nextResidualDot / residualDot;
        for (int i = 0; i < numVariables; ++i) {
            direction[i] = preconditioned[i] + beta * direction[i];
        }
        residualDot = nextResidualDot;
    }
    return solution;
}

// Surface sampling and remeshing stay native. The exported mesh can reorder
// triangles into material runs, so build its physical halfedges from explicit
// merge IDs, never by welding nearby coordinates or matching property indices.
namespace SurfaceUV {

using Point = linalg::vec<double, 3>;
using Point2 = linalg::vec<double, 2>;

double Cross2(Point2 a, Point2 b) { return a.x * b.y - a.y * b.x; }

Point Unit(Point p, const char* name) {
    double length = linalg::length(p);
    if (!std::isfinite(length) || length < 1e-12)
        throw std::invalid_argument(std::string(name) + " must be finite and nonzero");
    return p / length;
}

struct Face {
    std::array<int, 3> vertices;
    Point normal;
};

struct Cursor {
    int face;
    Point point;
};

struct Ray {
    Cursor cursor;
    Point direction;
    double distance;
};

struct Topology {
    manifold::MeshGL64 mesh;
    std::vector<Point> positions;
    std::vector<Face> faces;
    std::vector<int> pair;
    double epsilon;

    explicit Topology(const manifold::Manifold& man) : mesh(man.GetMeshGL64()) {
        if (!mesh.halfedgeTangent.empty())
            throw std::invalid_argument("Surface UV requires a triangle surface; Refine smooth tangents first");
        epsilon = std::max(man.GetEpsilon() * 4, 1e-10);
        UVUnionFind merged(mesh.NumVert());
        for (size_t i = 0; i < mesh.mergeFromVert.size(); ++i)
            merged.unite(mesh.mergeFromVert[i], mesh.mergeToVert[i]);
        for (size_t i = 0; i < mesh.NumVert(); ++i)
            positions.push_back(mesh.GetVertPos(i));
        std::unordered_map<uint64_t, int> edges;
        pair.assign(mesh.triVerts.size(), -1);
        for (size_t f = 0; f < mesh.NumTri(); ++f) {
            Face face;
            for (int c = 0; c < 3; ++c)
                face.vertices[c] = merged.find(mesh.triVerts[3 * f + c]);
            Point a = positions[face.vertices[0]];
            Point b = positions[face.vertices[1]];
            Point c = positions[face.vertices[2]];
            face.normal = Unit(linalg::cross(b - a, c - a), "face normal");
            faces.push_back(face);
            for (int c = 0; c < 3; ++c) {
                uint32_t a = face.vertices[c], b = face.vertices[(c + 1) % 3];
                uint64_t reverse = (uint64_t(b) << 32) | a;
                auto found = edges.find(reverse);
                int h = 3 * f + c;
                if (found != edges.end()) {
                    pair[h] = found->second;
                    pair[found->second] = h;
                    edges.erase(found);
                } else {
                    edges.emplace((uint64_t(a) << 32) | b, h);
                }
            }
        }
        if (!edges.empty())
            throw std::invalid_argument("Surface UV requires closed, paired halfedges");
    }

    Point Bary(int face, Point p) const {
        const auto& v = faces[face].vertices;
        Point a = positions[v[0]], ab = positions[v[1]] - a, ac = positions[v[2]] - a;
        Point n = linalg::cross(ab, ac);
        double denominator = linalg::dot(n, n);
        double b = linalg::dot(linalg::cross(p - a, ac), n) / denominator;
        double c = linalg::dot(linalg::cross(ab, p - a), n) / denominator;
        return {1 - b - c, b, c};
    }

    double BaryTolerance(int face) const {
        const auto& v = faces[face].vertices;
        Point a = positions[v[0]], b = positions[v[1]], c = positions[v[2]];
        double longest = std::max({linalg::length(b-a), linalg::length(c-b), linalg::length(a-c)});
        return epsilon * longest / linalg::length(linalg::cross(b-a, c-a));
    }

    Cursor Closest(Point p) const {
        Cursor result{-1, {}};
        double best = std::numeric_limits<double>::infinity();
        for (size_t f = 0; f < faces.size(); ++f) {
            const auto& face = faces[f];
            Point a = positions[face.vertices[0]];
            Point q = p - linalg::dot(p - a, face.normal) * face.normal;
            Point bary = Bary(f, q);
            if (std::min({bary.x, bary.y, bary.z}) < 0) {
                double edgeBest = std::numeric_limits<double>::infinity();
                for (int c = 0; c < 3; ++c) {
                    Point x = positions[face.vertices[c]];
                    Point d = positions[face.vertices[(c + 1) % 3]] - x;
                    Point candidate = x + std::clamp(linalg::dot(p-x, d) / linalg::dot(d,d), 0.0, 1.0) * d;
                    double distance = linalg::length2(candidate - p);
                    if (distance < edgeBest) { edgeBest = distance; q = candidate; }
                }
            }
            double distance = linalg::length2(q - p);
            if (distance < best) { best = distance; result = {int(f), q}; }
        }
        return result;
    }

    // At an ordinary crossing this visits two faces. At a vertex it follows
    // the incident halfedge fan until it finds the face the cut enters.
    Ray Enter(Cursor cursor, Point planeNormal, Point forward, Point chartNormal) const {
        std::vector<int> pending{cursor.face};
        std::unordered_set<int> seen;
        for (size_t i = 0; i < pending.size(); ++i) {
            int f = pending[i];
            if (!seen.insert(f).second) continue;
            Point b = Bary(f, cursor.point);
            double tolerance = BaryTolerance(f) * 4;
            if (std::min({b.x,b.y,b.z}) < -tolerance) continue;
            Point direction = linalg::cross(faces[f].normal, planeNormal);
            double length = linalg::length(direction);
            if (length > 1e-12 && linalg::dot(faces[f].normal, chartNormal) > 1e-6) {
                direction /= length;
                if (linalg::dot(direction, forward) < 0) direction = -direction;
                Point derivative = Bary(f, cursor.point + direction) - b;
                bool enters = linalg::dot(direction, forward) > 1e-6;
                double exit = std::numeric_limits<double>::infinity();
                for (int c = 0; c < 3; ++c) {
                    if (b[c] <= tolerance && derivative[c] < -1e-10) enters = false;
                    if (derivative[c] < -1e-12)
                        exit = std::min(exit, -std::max(0.0, b[c]) / derivative[c]);
                }
                if (enters && std::isfinite(exit) && exit > epsilon)
                    return {{f, cursor.point}, direction, exit};
            }
            for (int c = 0; c < 3; ++c)
                if (std::abs(b[(c+2)%3]) <= tolerance)
                    pending.push_back(pair[3*f+c] / 3);
        }
        throw std::invalid_argument("Surface UV walk reaches a fold or tangent; reduce the patch or change its orientation");
    }

    Cursor Walk(Cursor cursor, Point planeNormal, Point forward, Point normal, double distance) const {
        size_t crossings = 0;
        while (distance > epsilon) {
            if (++crossings > faces.size() * 2 + 8)
                throw std::runtime_error("Surface UV walk did not make progress");
            Ray ray = Enter(cursor, planeNormal, forward, normal);
            double step = std::min(distance, ray.distance);
            cursor = {ray.cursor.face, ray.cursor.point + step * ray.direction};
            distance -= step;
        }
        return cursor;
    }
};

struct Grid {
    Point origin, u, v, normal;
    std::vector<double> horizontal, vertical, x;
    std::vector<std::vector<double>> y;
    int seed;
    double minY, maxY;

    Point2 Project(Point p) const {
        p -= origin;
        return {linalg::dot(p,u), linalg::dot(p,v)};
    }

    static std::vector<double> Distances(double length, double step) {
        int count = int(std::ceil(length / step));
        std::vector<double> result;
        for (int i = 0; i < count; ++i) result.push_back(i * step);
        result.push_back(length);
        return result;
    }

    Grid(const Topology& topology, Point center, Point n, Point right,
         double width, double height, double pixelSize) {
        Cursor start = topology.Closest(center);
        seed = start.face;
        origin = start.point;
        normal = Unit(linalg::length(n) < 1e-12 ? topology.faces[seed].normal : n, "normal");
        u = Unit(right - linalg::dot(right, normal)*normal, "u direction");
        v = Unit(linalg::cross(normal,u), "v direction");
        horizontal = Distances(width, pixelSize);
        vertical = Distances(height, pixelSize);
        Cursor baseline = topology.Walk(start, v, -u, normal, width/2);
        minY = std::numeric_limits<double>::infinity();
        maxY = -minY;
        for (size_t i = 0; i < horizontal.size(); ++i) {
            if (i) baseline = topology.Walk(baseline, v, u, normal, horizontal[i]-horizontal[i-1]);
            x.push_back(Project(baseline.point).x);
            if (i && x[i] - x[i-1] <= topology.epsilon)
                throw std::invalid_argument("Surface UV columns fold over");
            Cursor column = topology.Walk(baseline, u, -v, normal, height/2);
            std::vector<double> samples;
            for (size_t j = 0; j < vertical.size(); ++j) {
                if (j) column = topology.Walk(column, u, v, normal, vertical[j]-vertical[j-1]);
                double y = Project(column.point).y;
                if (j && y-samples.back() <= topology.epsilon)
                    throw std::invalid_argument("Surface UV rows fold over");
                samples.push_back(y);
                minY = std::min(minY, y);
                maxY = std::max(maxY, y);
            }
            y.push_back(std::move(samples));
        }
    }
};

using Polygon = std::vector<Point2>;

// Convex clipping supplies both sides of each grid constraint, including the
// outside polygons. Coordinates are interpolated within the ORIGINAL face.
Polygon Clip(const Polygon& polygon, Point2 a, Point2 b, bool left, double epsilon) {
    Polygon output;
    if (polygon.empty()) return output;
    Point2 direction = b-a;
    double scale = linalg::length(direction);
    auto side = [&](Point2 p) {
        double d = Cross2(direction, p-a) / scale * (left ? 1 : -1);
        return std::abs(d) <= epsilon ? 0.0 : d;
    };
    Point2 previous = polygon.back();
    double before = side(previous);
    for (Point2 current : polygon) {
        double after = side(current);
        if ((before < 0 && after > 0) || (before > 0 && after < 0))
            output.push_back(previous + (current-previous)*(before/(before-after)));
        if (after >= 0) output.push_back(current);
        previous = current;
        before = after;
    }
    Polygon clean;
    for (Point2 p : output)
        if (clean.empty() || linalg::length(p-clean.back()) > epsilon) clean.push_back(p);
    if (clean.size() > 1 && linalg::length(clean.front()-clean.back()) <= epsilon) clean.pop_back();
    return clean.size() < 3 ? Polygon{} : clean;
}

double Area(const Polygon& polygon) {
    double sum = 0;
    for (size_t i = 1; i+1 < polygon.size(); ++i)
        sum += Cross2(polygon[i]-polygon[0], polygon[i+1]-polygon[0]);
    return sum/2;
}

struct ChartTriangle {
    std::array<Point2,3> point;
    std::array<Point2,3> uv;
    bool textured = false;
    uint64_t region = 0;

    Point2 Evaluate(Point2 p, Point2 outside) const {
        if (!textured) return outside;
        double denominator = Cross2(point[1]-point[0], point[2]-point[0]);
        double b = Cross2(p-point[0], point[2]-point[0])/denominator;
        double c = Cross2(point[1]-point[0], p-point[0])/denominator;
        return uv[0] + b*(uv[1]-uv[0]) + c*(uv[2]-uv[0]);
    }
};

struct Piece {
    int face;
    std::vector<int> vertices;
    ChartTriangle chart;
};

struct Remesh {
    const Topology& topology;
    const Grid& grid;
    std::vector<Point> positions;
    // Edge points have topology identities; coincident disconnected shells
    // and different property vertices are never accidentally welded.
    std::unordered_map<int, std::map<double,int>> edgePoints;
    std::vector<std::map<std::pair<int64_t,int64_t>,std::vector<int>>> interior;
    std::vector<std::unordered_map<int,std::map<double,int>>> columnPoints;
    std::vector<std::vector<Piece>> pieces;
    double coveredArea = 0;

    Remesh(const Topology& t, const Grid& g)
        : topology(t), grid(g), positions(t.positions), interior(t.faces.size()),
          columnPoints(t.faces.size()), pieces(t.faces.size()) {}

    int Edge(int face, int opposite) const {
        int h = 3*face+(opposite+1)%3;
        return std::min(h,topology.pair[h]);
    }

    double EdgeParameter(int edge, Point p) const {
        const Face& face = topology.faces[edge/3];
        Point a = topology.positions[face.vertices[edge%3]];
        Point d = topology.positions[face.vertices[(edge%3+1)%3]]-a;
        return linalg::dot(p-a,d)/linalg::dot(d,d);
    }

    int Vertex(int face, Point p) {
        Point b = topology.Bary(face,p);
        double tolerance = topology.BaryTolerance(face)*4;
        const auto& corners = topology.faces[face].vertices;
        for (int c = 0; c < 3; ++c)
            if (b[c] >= 1-tolerance) return corners[c];
        for (int c = 0; c < 3; ++c) {
            if (std::abs(b[c]) > tolerance) continue;
            int edge = Edge(face,c);
            const Face& f = topology.faces[edge/3];
            Point a = topology.positions[f.vertices[edge%3]];
            Point d = topology.positions[f.vertices[(edge%3+1)%3]]-a;
            double t = std::clamp(EdgeParameter(edge,p),0.0,1.0);
            double tol = topology.epsilon*4/linalg::length(d);
            auto& points = edgePoints[edge];
            auto next = points.lower_bound(t-tol);
            if (next != points.end() && std::abs(next->first-t) <= tol) return next->second;
            int index = positions.size();
            positions.push_back(a+t*d);
            points.emplace(t,index);
            return index;
        }
        Point2 xy = grid.Project(p);
        double quantum = topology.epsilon*4;
        int64_t x = std::llround(xy.x/quantum), y = std::llround(xy.y/quantum);
        auto& lookup = interior[face];
        for (int dx=-1; dx<=1; ++dx) for (int dy=-1; dy<=1; ++dy) {
            auto found = lookup.find({x+dx,y+dy});
            if (found != lookup.end()) for (int index : found->second)
                if (linalg::length(positions[index]-p) <= quantum) return index;
        }
        int index = positions.size();
        positions.push_back(p);
        lookup[{x,y}].push_back(index);
        return index;
    }

    Point Lift(int face, Point2 p) const {
        const auto& c = topology.faces[face].vertices;
        Point a = topology.positions[c[0]], b = topology.positions[c[1]], d = topology.positions[c[2]];
        Point2 x = grid.Project(a), y = grid.Project(b), z = grid.Project(d);
        double denominator = Cross2(y-x,z-x);
        double w1 = Cross2(p-x,z-x)/denominator, w2 = Cross2(y-x,p-x)/denominator;
        return a + w1*(b-a) + w2*(d-a);
    }

    void Add(int face, const Polygon& polygon, const ChartTriangle& chart = {}) {
        if (polygon.size() < 3 || Area(polygon) <= topology.epsilon*topology.epsilon) return;
        Piece piece{face,{},chart};
        for (Point2 p : polygon) {
            int id = Vertex(face,Lift(face,p));
            if (piece.vertices.empty() || piece.vertices.back()!=id) piece.vertices.push_back(id);
        }
        if (piece.vertices.size()>1 && piece.vertices.front()==piece.vertices.back()) piece.vertices.pop_back();
        if (piece.vertices.size()<3) return;
        for (int id:piece.vertices) {
            Point2 p=grid.Project(positions[id]);
            auto column=std::lower_bound(grid.x.begin(),grid.x.end(),p.x-topology.epsilon*8);
            if (column!=grid.x.end() && std::abs(*column-p.x)<=topology.epsilon*8)
                columnPoints[face][column-grid.x.begin()].emplace(p.y,id);
        }
        if (chart.textured) coveredArea += Area(polygon);
        pieces[face].push_back(std::move(piece));
    }

    void SplitFace(int face) {
        Polygon original;
        for (int c : topology.faces[face].vertices) original.push_back(grid.Project(topology.positions[c]));
        double epsilon = topology.epsilon;
        double firstX = grid.x.front(), lastX = grid.x.back();
        Add(face, Clip(original,{firstX,0},{firstX,1},true,epsilon),{{},{},false,1});
        Add(face, Clip(original,{lastX,0},{lastX,1},false,epsilon),{{},{},false,2});
        double minX = std::min({original[0].x,original[1].x,original[2].x});
        double maxX = std::max({original[0].x,original[1].x,original[2].x});
        size_t begin = std::max(ptrdiff_t(0), std::upper_bound(grid.x.begin(),grid.x.end(),minX)-grid.x.begin()-1);
        for (size_t i=begin; i+1<grid.x.size() && grid.x[i]<maxX+epsilon; ++i) {
            double a = grid.x[i], b = grid.x[i+1];
            Polygon strip = Clip(Clip(original,{a,0},{a,1},false,epsilon),{b,0},{b,1},true,epsilon);
            if (strip.empty()) continue;
            Point2 bottomA{a,grid.y[i][0]}, bottomB{b,grid.y[i+1][0]};
            Add(face,Clip(strip,bottomA,bottomB,false,epsilon),{{},{},false,3});
            Polygon remaining = Clip(strip,bottomA,bottomB,true,epsilon);
            for (size_t j=0; j+1<grid.vertical.size() && !remaining.empty(); ++j) {
                Point2 p00{a,grid.y[i][j]}, p10{b,grid.y[i+1][j]};
                Point2 p01{a,grid.y[i][j+1]}, p11{b,grid.y[i+1][j+1]};
                Polygon cell = Clip(remaining,p01,p11,false,epsilon);
                remaining = Clip(remaining,p01,p11,true,epsilon);
                if (cell.empty()) continue;
                double u0=grid.horizontal[i]/grid.horizontal.back(), u1=grid.horizontal[i+1]/grid.horizontal.back();
                // glTF images start at the top; positive surface V goes up.
                double v0=1-grid.vertical[j]/grid.vertical.back(), v1=1-grid.vertical[j+1]/grid.vertical.back();
                uint64_t region=5+2*(i*(grid.vertical.size()-1)+j);
                ChartTriangle lower{{p00,p10,p11},{{{u0,v0},{u1,v0},{u1,v1}}},true,region};
                ChartTriangle upper{{p00,p11,p01},{{{u0,v0},{u1,v1},{u0,v1}}},true,region+1};
                Add(face,Clip(cell,p00,p11,false,epsilon),lower);
                Add(face,Clip(cell,p00,p11,true,epsilon),upper);
            }
            Add(face,remaining,{{},{},false,4});
        }
    }

    void Build() {
        std::vector<bool> visited(topology.faces.size(),false);
        std::vector<int> pending{grid.seed};
        for (size_t i=0; i<pending.size(); ++i) {
            int f = pending[i];
            if (visited[f]) continue;
            visited[f] = true;
            bool left=true,right=true,below=true,above=true;
            for (int c : topology.faces[f].vertices) {
                Point2 p = grid.Project(topology.positions[c]);
                left &= p.x < grid.x.front()-topology.epsilon;
                right &= p.x > grid.x.back()+topology.epsilon;
                below &= p.y < grid.minY-topology.epsilon;
                above &= p.y > grid.maxY+topology.epsilon;
            }
            if (left || right || below || above) continue;
            if (linalg::dot(topology.faces[f].normal,grid.normal)<=1e-6) continue;
            SplitFace(f);
            for (int c=0;c<3;++c) pending.push_back(topology.pair[3*f+c]/3);
        }
        double expected = 0;
        for (size_t i=0;i+1<grid.x.size();++i)
            expected += (grid.x[i+1]-grid.x[i]) *
                ((grid.y[i].back()-grid.y[i].front())+(grid.y[i+1].back()-grid.y[i+1].front()))/2;
        if (std::abs(coveredArea-expected) > std::max(expected*1e-7,topology.epsilon*100))
            throw std::invalid_argument("Surface UV patch has missing or overlapping surface coverage");
        for (size_t f=0; f<topology.faces.size(); ++f) {
            if (pieces[f].empty()) {
                const auto& c = topology.faces[f].vertices;
                pieces[f].push_back({int(f),{c[0],c[1],c[2]},{}});
            }
        }
    }

    // Propagate every original-edge insertion to both incident faces, including
    // faces outside the patch. This is what prevents T-junctions and cracks.
    std::vector<int> Boundary(const Piece& piece) const {
        std::vector<int> result;
        double tolerance=topology.BaryTolerance(piece.face)*8;
        for (size_t i=0;i<piece.vertices.size();++i) {
            int a=piece.vertices[i], b=piece.vertices[(i+1)%piece.vertices.size()];
            result.push_back(a);
            std::vector<std::pair<double,int>> middle;
            Point ba=topology.Bary(piece.face,positions[a]), bb=topology.Bary(piece.face,positions[b]);
            for (int c=0;c<3;++c) {
                if (std::abs(ba[c])>tolerance || std::abs(bb[c])>tolerance) continue;
                int edge=Edge(piece.face,c);
                auto found=edgePoints.find(edge);
                if (found==edgePoints.end()) continue;
                double ta=EdgeParameter(edge,positions[a]), tb=EdgeParameter(edge,positions[b]);
                for (auto it=found->second.upper_bound(std::min(ta,tb)); it!=found->second.end() && it->first<std::max(ta,tb); ++it)
                    if (it->second!=a && it->second!=b) middle.emplace_back((it->first-ta)/(tb-ta),it->second);
                break;
            }
            // The outer column abuts a single background polygon. Its edge
            // must contain all pixel samples from the many cells on the other
            // side, even when the entire sticker fits inside one input face.
            Point2 pa=grid.Project(positions[a]),pb=grid.Project(positions[b]);
            if (std::abs(pa.x-pb.x)<=topology.epsilon*8 && std::abs(pa.y-pb.y)>topology.epsilon) {
                auto column=std::lower_bound(grid.x.begin(),grid.x.end(),pa.x-topology.epsilon*8);
                if (column!=grid.x.end() && std::abs(*column-pa.x)<=topology.epsilon*8) {
                    auto found=columnPoints[piece.face].find(column-grid.x.begin());
                    if (found!=columnPoints[piece.face].end())
                        for (auto it=found->second.upper_bound(std::min(pa.y,pb.y));it!=found->second.end() && it->first<std::max(pa.y,pb.y);++it)
                            if (it->second!=a && it->second!=b) middle.emplace_back((it->first-pa.y)/(pb.y-pa.y),it->second);
                }
            }
            std::sort(middle.begin(),middle.end());
            for (const auto& entry:middle)
                if (result.back()!=entry.second) result.push_back(entry.second);
        }
        return result;
    }

    manifold::MeshGL64 Output(size_t propIndex, Point2 atlas, Point2 atlasSize, Point2 outside) {
        const auto& input=topology.mesh;
        manifold::MeshGL64 output;
        output.numProp=std::max(size_t(input.numProp),propIndex+2);
        output.tolerance=input.tolerance;
        std::vector<int64_t> master(positions.size(),-1);
        auto vertex=[&](const Piece& piece,int id) {
            Point p=positions[id];
            Point weights=topology.Bary(piece.face,p);
            size_t index=output.NumVert();
            Point2 uv=piece.chart.Evaluate(grid.Project(p),outside);
            if (piece.chart.textured) {
                uv.x=atlas.x+std::clamp(uv.x,0.0,1.0)*atlasSize.x;
                uv.y=atlas.y+std::clamp(uv.y,0.0,1.0)*atlasSize.y;
            }
            for (size_t c=0;c<output.numProp;++c) {
                double value=0;
                if (c<3) value=p[c];
                else if (c<input.numProp) for (int k=0;k<3;++k)
                    value+=weights[k]*input.vertProperties[input.triVerts[3*piece.face+k]*input.numProp+c];
                if (c==propIndex) value=uv.x;
                if (c==propIndex+1) value=uv.y;
                output.vertProperties.push_back(value);
            }
            if (master[id]<0) master[id]=index;
            else { output.mergeFromVert.push_back(index); output.mergeToVert.push_back(master[id]); }
            return uint64_t(index);
        };
        auto triangle=[&](int face,uint64_t a,uint64_t b,uint64_t c) {
            output.triVerts.insert(output.triVerts.end(),{a,b,c});
            if (!input.faceID.empty()) output.faceID.push_back(input.faceID[face]);
        };
        auto emit=[&](const Piece& piece) {
                int f=piece.face;
                std::vector<int> boundary=Boundary(piece);
                std::vector<uint64_t> corners;
                for (int id:boundary) corners.push_back(vertex(piece,id));
                if (corners.size()==3) triangle(f,corners[0],corners[1],corners[2]);
                else {
                    // A center fan retains collinear samples on constrained
                    // edges, including pixel positions inside original faces.
                    Point center{0,0,0};
                    for (int id:boundary) center+=positions[id];
                    center/=double(boundary.size());
                    int id=positions.size();
                    positions.push_back(center); master.push_back(-1);
                    uint64_t mid=vertex(piece,id);
                    for (size_t k=0;k<corners.size();++k)
                        triangle(f,mid,corners[k],corners[(k+1)%corners.size()]);
                }
        };
        // In this Manifold version, coplanar cleanup can merge triangles with
        // different UV derivatives, or collapse a sticker corner surrounded by
        // just two faces. Partition each source instance into chart regions.
        // Repeated original IDs and transforms preserve source/material lookup,
        // while distinct run instances protect the affine charts through later
        // booleans. In particular, background sides meet in distinct regions.
        for (size_t run=0;run<input.runOriginalID.size();++run) {
            std::map<uint64_t,std::vector<const Piece*>> regions;
            for (size_t f=input.runIndex[run]/3;f<input.runIndex[run+1]/3;++f)
                for (const Piece& piece:pieces[f]) regions[piece.chart.region].push_back(&piece);
            if (regions.empty()) regions[0]={};
            for (const auto& entry:regions) {
                output.runIndex.push_back(output.triVerts.size());
                output.runOriginalID.push_back(input.runOriginalID[run]);
                if (!input.runTransform.empty())
                    output.runTransform.insert(output.runTransform.end(),
                        input.runTransform.begin()+12*run,input.runTransform.begin()+12*(run+1));
                for (const Piece* piece:entry.second) emit(*piece);
            }
        }
        output.runIndex.push_back(output.triVerts.size());
        return output;
    }
};

}  // namespace SurfaceUV

/**
 * Sample plane/surface intersections at physical pixel spacing, then overlay
 * the sampled grid onto the original faces. Only the local patch and its
 * boundary neighbors are retriangulated. UV seams use separate property
 * vertices with explicit physical merges, preserving boolean compatibility.
 *
 * This is a local plane-cut parameterization, not a shortest-geodesic solver.
 * It requires a single surface sheet over the seed tangent plane; folds,
 * grazing cuts and incomplete patches fail explicitly.
 */
manifold::Manifold GeodesicUV(const manifold::Manifold& man, size_t propIndex,
                              double originX, double originY, double originZ,
                              double normalX, double normalY, double normalZ,
                              double uDirectionX, double uDirectionY, double uDirectionZ,
                              double sizeU, double sizeV,
                              double atlasU, double atlasV, double atlasWidth, double atlasHeight,
                              double outsideU, double outsideV, double pixelSize) {
    for (double value : {originX,originY,originZ,normalX,normalY,normalZ,
                         uDirectionX,uDirectionY,uDirectionZ,sizeU,sizeV,
                         atlasU,atlasV,atlasWidth,atlasHeight,outsideU,outsideV,pixelSize})
        if (!std::isfinite(value)) throw std::invalid_argument("Surface UV parameters must be finite");
    if (propIndex<3 || propIndex>size_t(std::numeric_limits<int>::max()-2))
        throw std::invalid_argument("UV property index must be at least 3 and fit an int");
    if (sizeU<=0 || sizeV<=0 || pixelSize<=0 || atlasWidth<=0 || atlasHeight<=0)
        throw std::invalid_argument("Surface UV size, pixel size and atlas size must be positive");
    double columns=std::ceil(sizeU/pixelSize)+1, rows=std::ceil(sizeV/pixelSize)+1;
    if (columns*rows>2000000)
        throw std::invalid_argument("Surface UV grid exceeds two million samples; increase pixel size");
    if (man.IsEmpty()) return man;
    SurfaceUV::Topology topology(man);
    if (pixelSize<=topology.epsilon*32)
        throw std::invalid_argument("Surface UV pixel size is below mesh precision");
    SurfaceUV::Grid grid(topology,{originX,originY,originZ},{normalX,normalY,normalZ},
                         {uDirectionX,uDirectionY,uDirectionZ},sizeU,sizeV,pixelSize);
    SurfaceUV::Remesh remesh(topology,grid);
    remesh.Build();
    manifold::Manifold result(remesh.Output(propIndex,{atlasU,atlasV},{atlasWidth,atlasHeight},{outsideU,outsideV}));
    if (result.Status()!=manifold::Manifold::Error::NoError || result.IsEmpty())
        throw std::runtime_error("Surface UV retriangulation failed manifold validation");
    return result;
}

/**
 * Unwrap a manifold into UV charts using a native LSCM solve.
 *
 * Curvature seams are introduced at edges whose dihedral angle exceeds
 * seamAngleDegrees. Closed components additionally receive a topology-aware
 * cut graph, built from a shortest primal tree and a dual spanning tree. Each
 * chart is solved globally with least-squares conformal mapping, then its
 * property vertices are split at chart boundaries and related with MeshGL's
 * merge vectors so Manifold can still perform boolean operations on the result.
 */
manifold::Manifold UnwrapUV(const manifold::Manifold& man,
                            size_t propIndex,
                            double seamAngleDegrees,
                            double scale,
                            double padding,
                            bool pack) {
    if (propIndex < 3) {
        throw std::invalid_argument("UV property index must be at least 3");
    }
    if (!std::isfinite(seamAngleDegrees) || seamAngleDegrees < 0.0 ||
        seamAngleDegrees > 180.0) {
        throw std::invalid_argument("UV seam angle must be in [0, 180]");
    }
    if (!std::isfinite(scale) || scale <= 0.0) {
        throw std::invalid_argument("UV scale must be positive and finite");
    }
    if (!std::isfinite(padding) || padding < 0.0 || padding >= 0.5) {
        throw std::invalid_argument("UV atlas padding must be in [0, 0.5)");
    }

    manifold::MeshGL mesh = man.GetMeshGL();
    const size_t numTriangles = mesh.NumTri();
    const size_t numInputVertices = mesh.NumVert();
    if (numTriangles == 0 || numInputVertices == 0) return man;

    const size_t oldNumProps = mesh.numProp;
    const size_t newNumProps = std::max(propIndex + 2, oldNumProps);
    std::vector<UVVec3> positions(numInputVertices);
    for (size_t vertex = 0; vertex < numInputVertices; ++vertex) {
        const size_t offset = vertex * oldNumProps;
        positions[vertex] = {mesh.vertProperties[offset],
                             mesh.vertProperties[offset + 1],
                             mesh.vertProperties[offset + 2]};
    }

    UVUnionFind vertexSets(numInputVertices);
    for (size_t i = 0; i < mesh.mergeFromVert.size(); ++i) {
        const uint32_t from = mesh.mergeFromVert[i];
        const uint32_t to = mesh.mergeToVert[i];
        if (from >= numInputVertices || to >= numInputVertices) {
            throw std::invalid_argument("MeshGL merge index is out of bounds");
        }
        vertexSets.unite(static_cast<int>(from), static_cast<int>(to));
    }
    std::vector<int> logicalVertex(numInputVertices);
    for (size_t vertex = 0; vertex < numInputVertices; ++vertex) {
        logicalVertex[vertex] = vertexSets.find(static_cast<int>(vertex));
    }

    const size_t numCorners = mesh.triVerts.size();
    if (numCorners != 3 * numTriangles) {
        throw std::invalid_argument("MeshGL triangle buffer is not triangular");
    }

    std::vector<UVVec3> logicalPositions(numInputVertices);
    std::vector<bool> hasLogicalPosition(numInputVertices, false);
    for (size_t vertex = 0; vertex < numInputVertices; ++vertex) {
        const int logical = logicalVertex[vertex];
        if (!hasLogicalPosition[logical]) {
            logicalPositions[logical] = positions[vertex];
            hasLogicalPosition[logical] = true;
        }
    }

    std::vector<UVEdgeRecord> edges;
    std::vector<std::array<int, 3>> faceEdges(numTriangles);
    for (auto& face : faceEdges) face = {-1, -1, -1};
    std::unordered_map<uint64_t, int> edgeLookup;
    edgeLookup.reserve(3 * numTriangles);

    auto edgeKey = [](int a, int b) {
        if (a > b) std::swap(a, b);
        return (static_cast<uint64_t>(static_cast<uint32_t>(a)) << 32) |
               static_cast<uint32_t>(b);
    };

    for (size_t face = 0; face < numTriangles; ++face) {
        for (int localEdge = 0; localEdge < 3; ++localEdge) {
            const size_t corner = 3 * face + localEdge;
            const int a = logicalVertex[mesh.triVerts[corner]];
            const int b = logicalVertex[mesh.triVerts[3 * face +
                                                     (localEdge + 1) % 3]];
            if (a == b) {
                throw std::invalid_argument("MeshGL contains a degenerate edge");
            }
            const uint64_t key = edgeKey(a, b);
            auto found = edgeLookup.find(key);
            if (found == edgeLookup.end()) {
                const int edge = static_cast<int>(edges.size());
                edgeLookup.emplace(key, edge);
                edges.push_back({std::min(a, b), std::max(a, b),
                                 static_cast<int>(face), localEdge,
                                 -1, -1, false});
                faceEdges[face][localEdge] = edge;
            } else {
                UVEdgeRecord& edge = edges[found->second];
                if (edge.face1 >= 0) {
                    throw std::invalid_argument("MeshGL has a non-manifold edge");
                }
                edge.face1 = static_cast<int>(face);
                edge.edge1 = localEdge;
                faceEdges[face][localEdge] = found->second;
            }
        }
    }

    const double pi = std::acos(-1.0);
    const double seamCosine =
        std::cos(seamAngleDegrees * pi / 180.0);
    std::vector<UVVec3> faceNormals(numTriangles);
    std::vector<bool> validNormal(numTriangles, false);
    for (size_t face = 0; face < numTriangles; ++face) {
        const int v0 = logicalVertex[mesh.triVerts[3 * face]];
        const int v1 = logicalVertex[mesh.triVerts[3 * face + 1]];
        const int v2 = logicalVertex[mesh.triVerts[3 * face + 2]];
        const UVVec3 normal = UVCross(UVSub(logicalPositions[v1], logicalPositions[v0]),
                                      UVSub(logicalPositions[v2], logicalPositions[v0]));
        const double length = UVLength(normal);
        if (length > 1e-14) {
            faceNormals[face] = {normal.x / length, normal.y / length,
                                 normal.z / length};
            validNormal[face] = true;
        }
    }

    for (UVEdgeRecord& edge : edges) {
        if (edge.face1 < 0 || !validNormal[edge.face0] ||
            !validNormal[edge.face1]) {
            edge.seam = true;
            continue;
        }
        double cosine = UVDot(faceNormals[edge.face0], faceNormals[edge.face1]);
        cosine = std::max(-1.0, std::min(1.0, cosine));
        edge.seam = cosine < seamCosine;
    }

    auto otherFace = [&edges](int edgeIndex, int face) {
        const UVEdgeRecord& edge = edges[edgeIndex];
        return edge.face0 == face ? edge.face1 : edge.face0;
    };

    std::vector<int> initialComponent(numTriangles, -1);
    std::vector<std::vector<int>> components;
    for (size_t seed = 0; seed < numTriangles; ++seed) {
        if (initialComponent[seed] >= 0) continue;
        const int component = static_cast<int>(components.size());
        components.push_back({});
        std::queue<int> pending;
        pending.push(static_cast<int>(seed));
        initialComponent[seed] = component;
        while (!pending.empty()) {
            const int face = pending.front();
            pending.pop();
            components[component].push_back(face);
            for (int localEdge = 0; localEdge < 3; ++localEdge) {
                const int edgeIndex = faceEdges[face][localEdge];
                const UVEdgeRecord& edge = edges[edgeIndex];
                if (edge.seam || edge.face1 < 0) continue;
                const int neighbor = otherFace(edgeIndex, face);
                if (initialComponent[neighbor] < 0) {
                    initialComponent[neighbor] = component;
                    pending.push(neighbor);
                }
            }
        }
    }

    // A shortest primal tree plus a dual spanning tree gives a compact cut
    // graph for closed components. This is the topological part that turns
    // spheres and higher-genus components into parameterizable charts without
    // cutting every triangle edge.
    for (size_t component = 0; component < components.size(); ++component) {
        bool closed = true;
        for (const UVEdgeRecord& edge : edges) {
            if (edge.face0 < 0 ||
                initialComponent[edge.face0] != static_cast<int>(component)) {
                continue;
            }
            if (edge.seam || edge.face1 < 0 ||
                initialComponent[edge.face1] != static_cast<int>(component)) {
                closed = false;
                break;
            }
        }
        if (!closed) continue;

        std::vector<int> componentVertices;
        std::vector<bool> inComponent(numInputVertices, false);
        for (int face : components[component]) {
            for (int corner = 0; corner < 3; ++corner) {
                const int vertex = logicalVertex[mesh.triVerts[3 * face + corner]];
                if (!inComponent[vertex]) {
                    inComponent[vertex] = true;
                    componentVertices.push_back(vertex);
                }
            }
        }
        if (componentVertices.size() < 2) continue;

        std::vector<std::vector<std::pair<int, int>>> primal(numInputVertices);
        for (size_t edgeIndex = 0; edgeIndex < edges.size(); ++edgeIndex) {
            const UVEdgeRecord& edge = edges[edgeIndex];
            if (edge.face0 < 0 || edge.face1 < 0 ||
                initialComponent[edge.face0] != static_cast<int>(component) ||
                initialComponent[edge.face1] != static_cast<int>(component)) {
                continue;
            }
            const double length = std::max(
                1e-12, UVLength(UVSub(logicalPositions[edge.a],
                                      logicalPositions[edge.b])));
            primal[edge.a].push_back({edge.b, static_cast<int>(edgeIndex)});
            primal[edge.b].push_back({edge.a, static_cast<int>(edgeIndex)});
            (void)length;
        }

        const int root = componentVertices.front();
        const double infinity = std::numeric_limits<double>::infinity();
        std::vector<double> distance(numInputVertices, infinity);
        std::vector<int> parentEdge(numInputVertices, -1);
        std::priority_queue<std::pair<double, int>,
                            std::vector<std::pair<double, int>>,
                            std::greater<std::pair<double, int>>> queue;
        distance[root] = 0.0;
        queue.push({0.0, root});
        while (!queue.empty()) {
            const auto [currentDistance, vertex] = queue.top();
            queue.pop();
            if (currentDistance > distance[vertex]) continue;
            for (const auto& [neighbor, edgeIndex] : primal[vertex]) {
                const double edgeLength = std::max(
                    1e-12, UVLength(UVSub(logicalPositions[vertex],
                                          logicalPositions[neighbor])));
                const double candidate = currentDistance + edgeLength;
                if (candidate < distance[neighbor]) {
                    distance[neighbor] = candidate;
                    parentEdge[neighbor] = edgeIndex;
                    queue.push({candidate, neighbor});
                }
            }
        }

        std::vector<bool> treeEdge(edges.size(), false);
        for (int vertex : componentVertices) {
            if (vertex == root || parentEdge[vertex] < 0) continue;
            treeEdge[parentEdge[vertex]] = true;
            edges[parentEdge[vertex]].seam = true;
        }

        std::vector<bool> dualVisited(numTriangles, false);
        std::vector<bool> dualTreeEdge(edges.size(), false);
        std::queue<int> dualQueue;
        const int firstFace = components[component].front();
        dualVisited[firstFace] = true;
        dualQueue.push(firstFace);
        while (!dualQueue.empty()) {
            const int face = dualQueue.front();
            dualQueue.pop();
            for (int localEdge = 0; localEdge < 3; ++localEdge) {
                const int edgeIndex = faceEdges[face][localEdge];
                UVEdgeRecord& edge = edges[edgeIndex];
                if (edge.seam || edge.face1 < 0 ||
                    initialComponent[edge.face0] != static_cast<int>(component) ||
                    initialComponent[edge.face1] != static_cast<int>(component)) {
                    continue;
                }
                const int neighbor = otherFace(edgeIndex, face);
                if (!dualVisited[neighbor]) {
                    dualVisited[neighbor] = true;
                    dualTreeEdge[edgeIndex] = true;
                    dualQueue.push(neighbor);
                }
            }
        }

        for (size_t edgeIndex = 0; edgeIndex < edges.size(); ++edgeIndex) {
            UVEdgeRecord& edge = edges[edgeIndex];
            if (edge.face0 >= 0 && edge.face1 >= 0 &&
                initialComponent[edge.face0] == static_cast<int>(component) &&
                initialComponent[edge.face1] == static_cast<int>(component) &&
                !dualTreeEdge[edgeIndex]) {
                edge.seam = true;
            }
        }
    }

    std::vector<int> chartForFace(numTriangles, -1);
    std::vector<UVChartData> charts;
    for (size_t seed = 0; seed < numTriangles; ++seed) {
        if (chartForFace[seed] >= 0) continue;
        const int chartIndex = static_cast<int>(charts.size());
        charts.push_back({});
        std::queue<int> pending;
        pending.push(static_cast<int>(seed));
        chartForFace[seed] = chartIndex;
        while (!pending.empty()) {
            const int face = pending.front();
            pending.pop();
            charts[chartIndex].faces.push_back(face);
            for (int localEdge = 0; localEdge < 3; ++localEdge) {
                const int edgeIndex = faceEdges[face][localEdge];
                const UVEdgeRecord& edge = edges[edgeIndex];
                if (edge.seam || edge.face1 < 0) continue;
                const int neighbor = otherFace(edgeIndex, face);
                if (chartForFace[neighbor] < 0) {
                    chartForFace[neighbor] = chartIndex;
                    pending.push(neighbor);
                }
            }
        }
    }

    // Union face corners across non-seam edges. The resulting corner roots are
    // the parameterization vertices; seam sides remain separate even when
    // their underlying 3D positions are the same.
    UVUnionFind cornerSets(numCorners);
    for (const UVEdgeRecord& edge : edges) {
        if (edge.seam || edge.face1 < 0) continue;
        const int a0 = 3 * edge.face0 + edge.edge0;
        const int a1 = 3 * edge.face0 + (edge.edge0 + 1) % 3;
        const int b0 = 3 * edge.face1 + edge.edge1;
        const int b1 = 3 * edge.face1 + (edge.edge1 + 1) % 3;
        const int logicalA0 = logicalVertex[mesh.triVerts[a0]];
        const int logicalA1 = logicalVertex[mesh.triVerts[a1]];
        const int logicalB0 = logicalVertex[mesh.triVerts[b0]];
        const int logicalB1 = logicalVertex[mesh.triVerts[b1]];
        if (logicalA0 == logicalB0 && logicalA1 == logicalB1) {
            cornerSets.unite(a0, b0);
            cornerSets.unite(a1, b1);
        } else {
            cornerSets.unite(a0, b1);
            cornerSets.unite(a1, b0);
        }
    }

    std::vector<int> cornerRoot(numCorners);
    std::vector<int> cornerLogical(numCorners);
    std::vector<int> rootLogical(numCorners, -1);
    for (size_t corner = 0; corner < numCorners; ++corner) {
        cornerRoot[corner] = cornerSets.find(static_cast<int>(corner));
        cornerLogical[corner] = logicalVertex[mesh.triVerts[corner]];
        rootLogical[cornerRoot[corner]] = cornerLogical[corner];
    }

    for (UVChartData& chart : charts) {
        for (int face : chart.faces) {
            for (int corner = 0; corner < 3; ++corner) {
                const int root = cornerRoot[3 * face + corner];
                if (chart.local.find(root) == chart.local.end()) {
                    const int local = static_cast<int>(chart.vertices.size());
                    chart.local.emplace(root, local);
                    chart.vertices.push_back(root);
                }
            }
        }

        const int numVertices = static_cast<int>(chart.vertices.size());
        std::unordered_map<int, int> boundaryNext;
        std::unordered_map<int, double> boundaryLength;
        std::unordered_map<int, int> boundaryIncoming;
        bool simpleBoundary = true;
        for (int face : chart.faces) {
            for (int localEdge = 0; localEdge < 3; ++localEdge) {
                const UVEdgeRecord& edge = edges[faceEdges[face][localEdge]];
                if (!edge.seam && edge.face1 >= 0) continue;
                const int startCorner = 3 * face + localEdge;
                const int endCorner = 3 * face + (localEdge + 1) % 3;
                const int start = cornerRoot[startCorner];
                const int end = cornerRoot[endCorner];
                if (start == end) continue;
                auto nextFound = boundaryNext.find(start);
                if (nextFound != boundaryNext.end() && nextFound->second != end) {
                    simpleBoundary = false;
                } else {
                    boundaryNext[start] = end;
                    boundaryLength[start] = UVLength(UVSub(
                        logicalPositions[cornerLogical[endCorner]],
                        logicalPositions[cornerLogical[startCorner]]));
                }
                if (++boundaryIncoming[end] > 1) simpleBoundary = false;
            }
        }

        std::vector<int> boundary;
        if (simpleBoundary && !boundaryNext.empty()) {
            const int start = boundaryNext.begin()->first;
            int current = start;
            std::unordered_map<int, bool> visited;
            do {
                if (visited[current]) {
                    simpleBoundary = false;
                    break;
                }
                visited[current] = true;
                boundary.push_back(current);
                auto nextFound = boundaryNext.find(current);
                if (nextFound == boundaryNext.end()) {
                    simpleBoundary = false;
                    break;
                }
                current = nextFound->second;
            } while (current != start);
            if (current != start || visited.size() != boundaryNext.size()) {
                simpleBoundary = false;
            }
        }
        if (!simpleBoundary) boundary.clear();

        int anchorA = 0;
        int anchorB = numVertices > 1 ? 1 : 0;
        double greatestDistance = -1.0;
        for (int i = 1; i < numVertices; ++i) {
            const UVVec3 delta = UVSub(
                logicalPositions[rootLogical[chart.vertices[i]]],
                logicalPositions[rootLogical[chart.vertices[anchorA]]]);
            const double distance = UVDot(delta, delta);
            if (distance > greatestDistance) {
                greatestDistance = distance;
                anchorB = i;
            }
        }
        if (anchorB == anchorA && numVertices > 1) anchorB = 1;

        const UVVec3 anchorDelta = UVSub(
            logicalPositions[rootLogical[chart.vertices[anchorB]]],
            logicalPositions[rootLogical[chart.vertices[anchorA]]]);
        const double anchorLength = std::max(1e-6, UVLength(anchorDelta));
        std::vector<int> uVariable(numVertices, -1);
        std::vector<int> vVariable(numVertices, -1);
        std::vector<double> fixedU(numVertices, 0.0);
        std::vector<double> fixedV(numVertices, 0.0);
        std::vector<bool> fixedVertex(numVertices, false);
        if (boundary.size() >= 3) {
            double perimeter = 0.0;
            for (int root : boundary) perimeter += boundaryLength[root];
            perimeter = std::max(1e-12, perimeter);
            double distanceAlongBoundary = 0.0;
            for (int root : boundary) {
                const int vertex = chart.local.at(root);
                const double angle = 2.0 * pi * distanceAlongBoundary / perimeter;
                fixedVertex[vertex] = true;
                fixedU[vertex] = std::cos(angle);
                fixedV[vertex] = std::sin(angle);
                distanceAlongBoundary += boundaryLength[root];
            }
        } else {
            fixedVertex[anchorA] = true;
            fixedVertex[anchorB] = true;
            fixedU[anchorB] = anchorLength;
        }
        int numVariables = 0;
        for (int vertex = 0; vertex < numVertices; ++vertex) {
            if (fixedVertex[vertex]) continue;
            uVariable[vertex] = numVariables++;
            vVariable[vertex] = numVariables++;
        }

        std::vector<UVEquation> equations;
        equations.reserve(2 * chart.faces.size());
        auto addVariable = [&](UVEquation& equation, int localVertex,
                               bool isU, double coefficient) {
            const int variable = isU ? uVariable[localVertex]
                                     : vVariable[localVertex];
            if (variable >= 0) {
                equation.add(variable, coefficient);
            } else {
                equation.rhs -= coefficient *
                                (isU ? fixedU[localVertex]
                                     : fixedV[localVertex]);
            }
        };

        for (int face : chart.faces) {
            const int corner0 = 3 * face;
            const int corner1 = corner0 + 1;
            const int corner2 = corner0 + 2;
            const UVVec3 p0 = logicalPositions[cornerLogical[corner0]];
            const UVVec3 p1 = logicalPositions[cornerLogical[corner1]];
            const UVVec3 p2 = logicalPositions[cornerLogical[corner2]];
            const UVVec3 edge01 = UVSub(p1, p0);
            const UVVec3 edge02 = UVSub(p2, p0);
            const double length01 = UVLength(edge01);
            const double length02 = UVLength(edge02);
            if (length01 < 1e-12 || length02 < 1e-12) continue;
            const double localX1 = length01;
            const double localX2 = UVDot(edge01, edge02) / length01;
            const double localY2Squared =
                std::max(0.0, length02 * length02 - localX2 * localX2);
            const double localY2 = std::sqrt(localY2Squared);
            if (localY2 < 1e-12) continue;

            const double realPart[3] = {localX1 - localX2, localX2, -localX1};
            const double imaginaryPart[3] = {localY2, localY2, 0.0};
            const int locals[3] = {
                chart.local.at(cornerRoot[corner0]),
                chart.local.at(cornerRoot[corner1]),
                chart.local.at(cornerRoot[corner2])};

            UVEquation realEquation;
            UVEquation imaginaryEquation;
            for (int i = 0; i < 3; ++i) {
                addVariable(realEquation, locals[i], true, realPart[i]);
                addVariable(realEquation, locals[i], false, -imaginaryPart[i]);
                addVariable(imaginaryEquation, locals[i], true,
                            imaginaryPart[i]);
                addVariable(imaginaryEquation, locals[i], false,
                            realPart[i]);
            }
            if (realEquation.count > 0) equations.push_back(realEquation);
            if (imaginaryEquation.count > 0) equations.push_back(imaginaryEquation);
        }

        const std::vector<double> solution =
            SolveUVEquations(equations, numVariables);
        chart.uv.resize(numVertices);
        for (int vertex = 0; vertex < numVertices; ++vertex) {
            chart.uv[vertex].x = uVariable[vertex] >= 0
                                     ? solution[uVariable[vertex]]
                                     : fixedU[vertex];
            chart.uv[vertex].y = vVariable[vertex] >= 0
                                     ? solution[vVariable[vertex]]
                                     : fixedV[vertex];
            if (!std::isfinite(chart.uv[vertex].x) ||
                !std::isfinite(chart.uv[vertex].y)) {
                chart.uv[vertex] = {0.0, 0.0};
            }
        }
    }

    if (pack && !charts.empty()) {
        const int columns = std::max(
            1, static_cast<int>(std::ceil(std::sqrt(charts.size()))));
        const int rows = static_cast<int>((charts.size() + columns - 1) /
                                          columns);
        const double cellWidth = 1.0 / columns;
        const double cellHeight = 1.0 / rows;
        for (size_t chartIndex = 0; chartIndex < charts.size(); ++chartIndex) {
            UVChartData& chart = charts[chartIndex];
            double minU = std::numeric_limits<double>::infinity();
            double minV = std::numeric_limits<double>::infinity();
            double maxU = -std::numeric_limits<double>::infinity();
            double maxV = -std::numeric_limits<double>::infinity();
            for (const UVVec2& uv : chart.uv) {
                minU = std::min(minU, uv.x);
                minV = std::min(minV, uv.y);
                maxU = std::max(maxU, uv.x);
                maxV = std::max(maxV, uv.y);
            }
            const double width = std::max(1e-12, maxU - minU);
            const double height = std::max(1e-12, maxV - minV);
            const double usableWidth = cellWidth * (1.0 - 2.0 * padding);
            const double usableHeight = cellHeight * (1.0 - 2.0 * padding);
            const double chartScale =
                std::min(usableWidth / width, usableHeight / height);
            const int column = static_cast<int>(chartIndex) % columns;
            const int row = static_cast<int>(chartIndex) / columns;
            const double left = column * cellWidth + cellWidth * padding +
                                (usableWidth - chartScale * width) * 0.5;
            const double bottom = row * cellHeight + cellHeight * padding +
                                  (usableHeight - chartScale * height) * 0.5;
            for (UVVec2& uv : chart.uv) {
                uv.x = left + (uv.x - minU) * chartScale;
                uv.y = bottom + (uv.y - minV) * chartScale;
            }
        }
    } else {
        for (UVChartData& chart : charts) {
            for (UVVec2& uv : chart.uv) {
                uv.x *= scale;
                uv.y *= scale;
            }
        }
    }

    manifold::MeshGL output;
    output.numProp = newNumProps;
    output.tolerance = mesh.tolerance;
    output.faceID = mesh.faceID;
    output.runIndex = mesh.runIndex;
    output.runOriginalID = mesh.runOriginalID;
    output.runTransform = mesh.runTransform;
    output.halfedgeTangent = mesh.halfedgeTangent;
    output.triVerts.reserve(numCorners);
    output.vertProperties.reserve(numCorners * newNumProps);

    std::unordered_map<uint64_t, uint32_t> outputLookup;
    outputLookup.reserve(numCorners);
    std::vector<int> outputLogical;
    for (size_t corner = 0; corner < numCorners; ++corner) {
        const int face = static_cast<int>(corner / 3);
        const int root = cornerRoot[corner];
        const int local = charts[chartForFace[face]].local.at(root);
        const uint32_t sourceVertex = mesh.triVerts[corner];
        const uint64_t key =
            (static_cast<uint64_t>(sourceVertex) << 32) |
            static_cast<uint32_t>(root);
        auto found = outputLookup.find(key);
        uint32_t outputVertex;
        if (found == outputLookup.end()) {
            outputVertex = static_cast<uint32_t>(output.vertProperties.size() /
                                                  newNumProps);
            outputLookup.emplace(key, outputVertex);
            const size_t sourceOffset = sourceVertex * oldNumProps;
            const size_t outputOffset = output.vertProperties.size();
            for (size_t property = 0; property < newNumProps; ++property) {
                output.vertProperties.push_back(
                    property < oldNumProps
                        ? mesh.vertProperties[sourceOffset + property]
                        : 0.0f);
            }
            output.vertProperties[outputOffset + propIndex] =
                static_cast<float>(charts[chartForFace[face]].uv[local].x);
            output.vertProperties[outputOffset + propIndex + 1] =
                static_cast<float>(charts[chartForFace[face]].uv[local].y);
            outputLogical.push_back(logicalVertex[sourceVertex]);
        } else {
            outputVertex = found->second;
        }
        output.triVerts.push_back(outputVertex);
    }

    std::unordered_map<int, uint32_t> firstOutputForLogical;
    firstOutputForLogical.reserve(outputLogical.size());
    for (uint32_t vertex = 0; vertex < outputLogical.size(); ++vertex) {
        const int logical = outputLogical[vertex];
        auto found = firstOutputForLogical.find(logical);
        if (found == firstOutputForLogical.end()) {
            firstOutputForLogical.emplace(logical, vertex);
        } else if (found->second != vertex) {
            output.mergeFromVert.push_back(vertex);
            output.mergeToVert.push_back(found->second);
        }
    }

    return manifold::Manifold(output);
}


manifold::Manifold CreateSurface(const std::string& texturePath, double pixelWidth = 1.0) {
    // Load the texture image in its original format to determine channels
    int width, height, channels;
    unsigned char* data = stbi_load(texturePath.c_str(), &width, &height, &channels, 0);  // 0 keeps original channels
    if (!data) {
        throw std::runtime_error("Failed to load texture image.");
    }

    // Set the number of properties based on the image format
    int numProps = channels;  // 1 for grayscale, 3 for RGB, 4 for RGBA

    // Create a property map from the texture data
    std::vector<float> propertyMap(width * height * numProps);
    for (int i = 0; i < width * height; ++i) {
        for (int c = 0; c < channels; ++c) {
            propertyMap[i * numProps + c] = static_cast<float>(data[i * channels + c]) / 255.0f;  // Normalize to [0, 1]
        }
    }
    stbi_image_free(data); // Free the image data

    // Invoke the overloaded CreateSurface function with the property map and numProps
    return CreateSurface(propertyMap.data(), numProps, width, height, pixelWidth);
}

manifold::Manifold LoadImage(const std::string& texturePath, const float depth, double pixelWidth = 1.0) {
    // Load the texture image in its original format to determine channels
    int width, height, channels;
    unsigned char* data = stbi_load(texturePath.c_str(), &width, &height, &channels, 0);  // 0 keeps original channels
    if (!data) {
        throw std::runtime_error("Failed to load texture image.");
    }

    // Set the number of properties based on the image format
    int numProps = channels + 1;

    // Create a property map from the texture data
    std::vector<float> propertyMap(width * height * numProps);
    for (int i = 0; i < width * height; ++i) {
        propertyMap[i * numProps] = depth;
        for (int c = 1; c < channels; ++c) {
            propertyMap[i * numProps + c] = static_cast<float>(data[i * channels + c]) / 255.0f;  // Normalize to [0, 1]
        }
    }
    stbi_image_free(data); // Free the image data

    // Invoke the overloaded CreateSurface function with the property map and numProps
    return CreateSurface(propertyMap.data(), numProps, width, height, pixelWidth);
}

std::vector<ivec3> TriangulateFaces(const std::vector<vec3>& vertices, const std::vector<std::vector<uint32_t>>& faces, float precision) {
    std::vector<ivec3> result;
    for (const auto& face : faces) {
        // If the face only has 3 vertices, no need to triangulate, just add it to result
        if (face.size() == 3) {
            result.push_back(ivec3(face[0], face[1], face[2]));
            continue;
        }

        // Compute face normal
        vec3 normal = linalg::cross(vertices[face[1]] - vertices[face[0]], vertices[face[2]] - vertices[face[0]]);
        normal = linalg::normalize(normal);

        // Compute reference right vector
        vec3 right = linalg::normalize(vertices[face[1]] - vertices[face[0]]);

        // Compute up vector
        vec3 up = linalg::cross(right, normal);

        // Project vertices onto plane
        std::vector<vec2> face2D;
        for (const auto& index : face) {
            vec3 local = vertices[index] - vertices[face[0]];
            face2D.push_back(vec2(linalg::dot(local, right), linalg::dot(local, up)));
        }

        // Triangulate and remap the triangulated vertices back to the original indices
        std::vector<ivec3> triVerts = manifold::Triangulate({face2D}, precision);
        for (auto& tri : triVerts) {
            tri.x = face[tri.x];
            tri.y = face[tri.y];
            tri.z = face[tri.z];
        }

        // Append to result
        result.insert(result.end(), triVerts.begin(), triVerts.end());
    }
    return result;
}

manifold::Manifold Polyhedron(const std::vector<vec3>& vertices, const std::vector<std::vector<uint32_t>>& faces) {
    std::vector<ivec3> triVerts = TriangulateFaces(vertices, faces, -1.0);
    return CreateManifold(vertices, triVerts);
}

manifold::Manifold Polyhedron(double* vertices, std::size_t nVertices, int* faceBuf, int* faceLengths, std::size_t nFaces) {

    std::vector<vec3> verts = BufferUtils::createDoubleVec3Vector(vertices, nVertices*3);

    std::vector<std::vector<uint32_t>> faces;
    for (std::size_t faceIdx = 0, faceBufIndex = 0; faceIdx < nFaces; faceIdx++) {
        std::size_t faceLength = (std::size_t) faceLengths[faceIdx];
        std::vector<uint32_t> face;
        for (size_t j = 0; j < faceLength; j++) {
            face.push_back((uint32_t) faceBuf[faceBufIndex]);
            faceBufIndex++;
        }
        faces.push_back(face);
    }

    return Polyhedron(verts, faces);
}

enum class LoftAlgorithm: long {
   EagerNearestNeighbor,
   Isomorphic
};

vec2 calculatePolygonCentroid(const std::vector<vec2>& vertices) {
    if (vertices.size() < 3) {
        throw std::invalid_argument("A polygon must have at least 3 vertices.");
    }

    double centroidX = 0.0;
    double centroidY = 0.0;
    double signedArea = 0.0;
    double x0 = 0.0;
    double y0 = 0.0;
    double x1 = 0.0;
    double y1 = 0.0;
    double a = 0.0;  // Partial signed area

    size_t i = 0;
    for (i = 0; i < vertices.size() - 1; ++i) {
        x0 = vertices[i].x;
        y0 = vertices[i].y;
        x1 = vertices[i+1].x;
        y1 = vertices[i+1].y;
        a = x0*y1 - x1*y0;
        signedArea += a;
        centroidX += (x0 + x1) * a;
        centroidY += (y0 + y1) * a;
    }

    x0 = vertices[i].x;
    y0 = vertices[i].y;
    x1 = vertices[0].x;
    y1 = vertices[0].y;
    a = x0*y1 - x1*y0;
    signedArea += a;
    centroidX += (x0 + x1) * a;
    centroidY += (y0 + y1) * a;

    signedArea *= 0.5;
    centroidX /= (6.0 * signedArea);
    centroidY /= (6.0 * signedArea);

    return vec2(centroidX, centroidY);
}

manifold::Manifold EagerNearestNeighborLoft(const std::vector<manifold::Polygons>& sections, const std::vector<mat3x4>& transforms) {
    if (sections.size() != transforms.size()) {
      throw std::runtime_error("Mismatched number of sections and transforms");
    }
    if (sections.size() < 2) {
      throw std::runtime_error("Loft requires at least two sections.");
    }

    size_t nVerts = 0;
    std::vector<size_t> sectionSizes;
    sectionSizes.reserve(sections.size());
    for (auto& section: sections) {
        size_t sectionSize = 0;
        for (auto& poly: section) {
            sectionSize += poly.size();
        }
        nVerts += sectionSize;
        sectionSizes.push_back(sectionSize);
    }

    std::vector<vec3> vertPos;
    vertPos.reserve(nVerts);
    std::vector<ivec3> triVerts;
    triVerts.reserve(2*nVerts);

    size_t botSectionOffset = 0;
    for (std::size_t i = 0; i < sections.size() - 1; ++i) {
        const manifold::Polygons& botPolygons = sections[i];
        const manifold::Polygons& topPolygons = sections[i + 1];
        const mat3x4& botTransform = transforms[i];

        if (botPolygons.size() != topPolygons.size()) {
          throw std::runtime_error("Cross sections must be composed of euqal number of polygons.");
        }

        size_t botSectionSize = sectionSizes[i];
        size_t topSectionOffset = botSectionOffset + botSectionSize;

        size_t botPolyOffset = 0;
        size_t topPolyOffset = 0;
        auto currPolyIt = botPolygons.begin();
        auto nextPolyIt = topPolygons.begin();
        for (int idx = 0; currPolyIt != botPolygons.end(); idx++, currPolyIt++, nextPolyIt++) {
          auto botPolygon = *currPolyIt;
          auto topPolygon = *nextPolyIt;

          vec2 botCentroid = calculatePolygonCentroid(botPolygon);
          vec2 topCentroid = calculatePolygonCentroid(topPolygon);
          vec2 centroidOffset = topCentroid - botCentroid;

          for (const auto& vertex : botPolygon) {
              vertPos.push_back(MatrixTransforms::Translate(botTransform, vec3(vertex.x, vertex.y, 0))[3]);
          }

          float minDistance = std::numeric_limits<float>::max();
          size_t botStartVertOffset = 0,
            topStartVertOffset = 0;
          for (size_t j = 0; j < topPolygon.size(); ++j) {
            float dist = linalg::distance(botPolygon[0], topPolygon[j] - centroidOffset);
            if (dist < minDistance) {
              minDistance = dist;
              topStartVertOffset = j;
            }
          }

          bool botHasMoved = false,
            topHasMoved = false;
          size_t botVertOffset = botStartVertOffset,
            topVertOffset = topStartVertOffset;
          do {
              size_t botNextVertOffset = (botVertOffset + 1) % botPolygon.size();
              size_t topNextVertOffset = (topVertOffset + 1) % topPolygon.size();

              float distBotNextToTop = linalg::distance(botPolygon[botNextVertOffset], topPolygon[topVertOffset] - centroidOffset);
              float distBotToTopNext = linalg::distance(botPolygon[botVertOffset], topPolygon[topNextVertOffset] - centroidOffset);
              float distBotNextToTopNext = linalg::distance(botPolygon[botNextVertOffset], topPolygon[topNextVertOffset] - centroidOffset);

              bool botHasNext = botNextVertOffset != (botStartVertOffset + 1) % botPolygon.size() || !botHasMoved;
              bool topHasNext = topNextVertOffset != (topStartVertOffset + 1) % topPolygon.size() || !topHasMoved;

              if (distBotNextToTopNext < distBotNextToTop && distBotNextToTopNext <= distBotToTopNext && botHasNext && topHasNext) {
                  triVerts.emplace_back(botSectionOffset + botPolyOffset + botVertOffset,
                                        topSectionOffset + topPolyOffset + topNextVertOffset,
                                        topSectionOffset + topPolyOffset + topVertOffset);
                  triVerts.emplace_back(botSectionOffset + botPolyOffset + botVertOffset,
                                        botSectionOffset + botPolyOffset + botNextVertOffset,
                                        topSectionOffset + topPolyOffset + topNextVertOffset);
                  botVertOffset = botNextVertOffset;
                  topVertOffset = topNextVertOffset;
                  botHasMoved = true;
                  topHasMoved = true;
              } else if (distBotNextToTop < distBotToTopNext && botHasNext) {
                  triVerts.emplace_back(botSectionOffset + botPolyOffset + botVertOffset,
                                        botSectionOffset + botPolyOffset + botNextVertOffset,
                                        topSectionOffset + topPolyOffset + topVertOffset);
                  botVertOffset = botNextVertOffset;
                  botHasMoved = true;
              } else {
                  triVerts.emplace_back(botSectionOffset + botPolyOffset + botVertOffset,
                                        topSectionOffset + topPolyOffset + topNextVertOffset,
                                        topSectionOffset + topPolyOffset + topVertOffset);
                  topVertOffset = topNextVertOffset;
                  topHasMoved = true;
              }

          } while (botVertOffset != botStartVertOffset || topVertOffset != topStartVertOffset);
          botPolyOffset += botPolygon.size();
          topPolyOffset += topPolygon.size();
        }
        botSectionOffset += botSectionSize;
    }

    auto frontPolygons = sections.front();
    auto frontTriangles = manifold::Triangulate(frontPolygons, -1.0);
    for (auto& tri : frontTriangles) {
      triVerts.push_back({tri[2], tri[1], tri[0]});
    }

    auto backPolygons = sections.back();
    auto backTransform = transforms.back();
    for (const auto& poly: backPolygons) {
      for (const auto& vertex : poly) {
        vertPos.push_back(MatrixTransforms::Translate(backTransform, vec3(vertex.x, vertex.y, 0))[3]);
      }
    }
    auto backTriangles = manifold::Triangulate(backPolygons, -1.0);

    for (auto& triangle : backTriangles) {
        triangle[0] += botSectionOffset;
        triangle[1] += botSectionOffset;
        triangle[2] += botSectionOffset;
        triVerts.push_back(triangle);
    }

    auto man = CreateManifold(vertPos, triVerts);
    return man;
}

manifold::Manifold IsomorphicLoft(const std::vector<manifold::Polygons>& sections, const std::vector<mat3x4>& transforms) {
    std::vector<vec3> vertPos;
    std::vector<ivec3> triVerts;

    if (sections.size() != transforms.size()) {
        throw std::runtime_error("Mismatched number of sections and transforms");
    }

    std::size_t offset = 0;
    std::size_t nVerticesInEachSection = 0;

    for (std::size_t i = 0; i < sections.size(); ++i) {
        const manifold::Polygons polygons = sections[i];
        mat3x4 transform = transforms[i];

        for (const auto& polygon : polygons) {
            for (const vec2& vertex : polygon) {
                vec3 translatedVertex = MatrixTransforms::Translate(transform, vec3(vertex.x, vertex.y, 0))[3];
                vertPos.push_back(translatedVertex);
            }
        }

        if (i == 0) {
            nVerticesInEachSection = vertPos.size();
        } else if ((vertPos.size() % nVerticesInEachSection) != 0)  {
            throw std::runtime_error("Recieved CrossSection with different number of vertices");
        }

        if (i < sections.size() - 1) {
            std::size_t currentOffset = offset;
            std::size_t nextOffset = offset + nVerticesInEachSection;

            for (std::size_t j = 0; j < polygons.size(); ++j) {
                const auto& polygon = polygons[j];

                for (std::size_t k = 0; k < polygon.size(); ++k) {
                    std::size_t nextIndex = (k + 1) % polygon.size();

                    ivec3 triangle1(currentOffset + k, currentOffset + nextIndex, nextOffset + k);
                    ivec3 triangle2(currentOffset + nextIndex, nextOffset + nextIndex, nextOffset + k);

                    triVerts.push_back(triangle1);
                    triVerts.push_back(triangle2);
                }
                currentOffset += polygon.size();
                nextOffset += polygon.size();
            }
        }

        offset += nVerticesInEachSection;
    }

    auto frontPolygons = sections.front();
    auto frontTriangles = manifold::Triangulate(frontPolygons, -1.0);
    for (auto& tri : frontTriangles) {
        triVerts.push_back({tri.z, tri.y, tri.x});
    }

    auto backPolygons = sections.back();
    auto backTriangles = manifold::Triangulate(backPolygons, -1.0);
    for (auto& triangle : backTriangles) {
        triangle.x += offset - nVerticesInEachSection;
        triangle.y += offset - nVerticesInEachSection;
        triangle.z += offset - nVerticesInEachSection;
        triVerts.push_back(triangle);
    }
    return CreateManifold(vertPos, triVerts);
}

manifold::Manifold Loft(const std::vector<manifold::Polygons>& sections, const std::vector<mat3x4>& transforms, LoftAlgorithm algorithm) {
    switch (algorithm) {
        case LoftAlgorithm::EagerNearestNeighbor:
            return EagerNearestNeighborLoft(sections, transforms);
        case LoftAlgorithm::Isomorphic:
            return IsomorphicLoft(sections, transforms);
        default:
            return EagerNearestNeighborLoft(sections, transforms);
    }
}

manifold::Manifold Loft(const std::vector<manifold::Polygons>& sections, const std::vector<mat3x4>& transforms) {
    return EagerNearestNeighborLoft(sections, transforms);
}

manifold::Manifold Loft(const manifold::Polygons& sections, const std::vector<mat3x4>& transforms) {
    std::vector<manifold::Polygons> polys;
    for (auto& section : sections) {
        polys.push_back({section});
    }
    return Loft(polys, transforms);
}

manifold::Manifold Loft(const manifold::Polygons& sections, const std::vector<mat3x4>& transforms, LoftAlgorithm algorithm) {
    std::vector<manifold::Polygons> polys;
    for (auto& section : sections) {
        polys.push_back({section});
    }
    return Loft(polys, transforms, algorithm);
}

manifold::Manifold Loft(const manifold::SimplePolygon& section, const std::vector<mat3x4>& transforms) {
    std::vector<manifold::Polygons> polys;
    for (std::size_t i = 0; i < transforms.size(); i++) {
        polys.push_back({section});
    }
    return Loft(polys, transforms);
}

manifold::Manifold Loft(const manifold::SimplePolygon& section, const std::vector<mat3x4>& transforms, LoftAlgorithm algorithm) {
    std::vector<manifold::Polygons> polys;
    for (std::size_t i = 0; i < transforms.size(); i++) {
        polys.push_back({section});
    }
    return Loft(polys, transforms, algorithm);
}

manifold::Manifold Loft(const std::vector<manifold::CrossSection>& sections, const std::vector<mat3x4>& transforms) {
    std::vector<manifold::Polygons> polys;
    for (auto section : sections) {
        polys.push_back(section.ToPolygons());
    }
    return Loft(polys, transforms);
}

manifold::Manifold Loft(const std::vector<manifold::CrossSection>& sections, const std::vector<mat3x4>& transforms, LoftAlgorithm algorithm) {
    std::vector<manifold::Polygons> polys;
    for (auto section : sections) {
        polys.push_back(section.ToPolygons());
    }
    return Loft(polys, transforms, algorithm);
}

manifold::Manifold Loft(const manifold::CrossSection section, const std::vector<mat3x4>& transforms) {
    std::vector<manifold::Polygons> sections(transforms.size());
    auto polys = section.ToPolygons();
    for (std::size_t i = 0; i < transforms.size(); i++) {
        sections[i] = polys;
    }
    return Loft(sections, transforms);
}

manifold::Manifold Loft(const manifold::CrossSection section, const std::vector<mat3x4>& transforms, LoftAlgorithm algorithm) {
    std::vector<manifold::Polygons> sections(transforms.size());
    auto polys = section.ToPolygons();
    for (std::size_t i = 0; i < transforms.size(); i++) {
        sections[i] = polys;
    }
    return Loft(sections, transforms, algorithm);
}

}
