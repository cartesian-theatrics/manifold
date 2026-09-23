#pragma once

#include "manifold/linalg.h"    // https://github.com/sgorsten/linalg
#include <cmath>       // for std::sin, std::cos

namespace MatrixTransforms {

// ------------------------------------------------------------
// Type aliases for convenience (double-precision):
using vec2   = linalg::vec<double, 2>;
using vec3   = linalg::vec<double, 3>;
using mat2x2 = linalg::mat<double, 2, 2>;
using mat3x3 = linalg::mat<double, 3, 3>;
using mat3x4 = linalg::mat<double, 3, 4>;
using mat2x3 = linalg::mat<double, 2, 3>;

// ------------------------------------------------------------
// Rodrigues Rotation: rotate vector v around (unit) axis k by angle a.
inline vec3 rodrigues_rotation(const vec3 &v, const vec3 &k, double a)
{
    using std::sin;
    using std::cos;

    double c = cos(a);
    double s = sin(a);

    // v*cos(a) + (k x v)*sin(a) + k*(k·v)*(1 - cos(a))
    return v * c
         + linalg::cross(k, v) * s
         + k * (linalg::dot(k, v) * (1.0 - c));
}

// ------------------------------------------------------------
// Yaw: rotate columns [0], [2] about column [1].
inline mat3x4 Yaw(const mat3x4 &m, double a)
{
    auto c0 = m[0];
    auto c1 = m[1];
    auto c2 = m[2];
    auto c3 = m[3];

    auto d0 = rodrigues_rotation(c0, c1, a);
    auto d1 = c1;  // unchanged
    auto d2 = rodrigues_rotation(c2, c1, a);

    mat3x4 result = m;
    result[0] = d0;
    result[1] = d1;
    result[2] = d2;
    result[3] = c3;

    return result;
}

// Pitch: rotate columns [1], [2] about column [0].
inline mat3x4 Pitch(const mat3x4 &m, double a)
{
    auto c0 = m[0];
    auto c1 = m[1];
    auto c2 = m[2];
    auto c3 = m[3];

    auto d0 = c0;  // unchanged
    auto d1 = rodrigues_rotation(c1, c0, a);
    auto d2 = rodrigues_rotation(c2, c0, a);

    mat3x4 result = m;
    result[0] = d0;
    result[1] = d1;
    result[2] = d2;
    result[3] = c3;

    return result;
}

// Roll: rotate columns [0], [1] about column [2].
inline mat3x4 Roll(const mat3x4 &m, double a)
{
    auto c0 = m[0];
    auto c1 = m[1];
    auto c2 = m[2];
    auto c3 = m[3];

    auto d0 = rodrigues_rotation(c0, c2, a);
    auto d1 = rodrigues_rotation(c1, c2, a);
    auto d2 = c2; // unchanged

    mat3x4 result = m;
    result[0] = d0;
    result[1] = d1;
    result[2] = d2;
    result[3] = c3;

    return result;
}

// Rotate: rotate columns [0..2] around an arbitrary axis.
inline mat3x4 Rotate(const mat3x4 &m, const vec3 &axis, double a)
{
    auto c0 = m[0];
    auto c1 = m[1];
    auto c2 = m[2];
    auto c3 = m[3];

    mat3x4 result = m;
    result[0] = rodrigues_rotation(c0, axis, a);
    result[1] = rodrigues_rotation(c1, axis, a);
    result[2] = rodrigues_rotation(c2, axis, a);
    result[3] = c3;

    return result;
}

// ------------------------------------------------------------
// 2D rotation of a vector
inline vec2 RotateVec2(const vec2 &v, double angleRadians)
{
    using std::sin;
    using std::cos;

    double c = cos(angleRadians);
    double s = sin(angleRadians);

    return {
        c*v.x - s*v.y,
        s*v.x + c*v.y
    };
}

// Combined 3D rotation (pitch->yaw->roll) given angles.x/y/z
inline mat3x4 Rotate(const mat3x4 &m, const vec3 &angles)
{
    mat3x4 res = m;
    if (angles.x != 0.0) {
        res = Pitch(res, angles.x);
    }
    if (angles.y != 0.0) {
        res = Yaw(res, angles.y);
    }
    if (angles.z != 0.0) {
        res = Roll(res, angles.z);
    }
    return res;
}

// ------------------------------------------------------------
// 2D transform matrix is 2 rows × 3 columns => mat2x3
// Rotate basis vectors in 2D
inline mat2x3 Rotate(const mat2x3 &m, double angleRadians)
{
    // columns: [0]=xAxis, [1]=yAxis, [2]=translation
    auto xAxis = m[0];
    auto yAxis = m[1];
    auto trans = m[2];

    vec2 xRot = RotateVec2(xAxis, angleRadians);
    vec2 yRot = RotateVec2(yAxis, angleRadians);

    mat2x3 result = m;
    result[0] = xRot;
    result[1] = yRot;
    result[2] = trans;

    return result;
}

// ------------------------------------------------------------
// Replace the upper-left 3x3 with a given rotation
inline mat3x4 SetRotation(const mat3x4 &m, const mat3x3 &rotation)
{
    mat3x4 result = m;
    for (size_t i = 0; i < 3; ++i) {
        vec3 col = rotation[i]; // each column is vec3
        result[i] = col;
    }
    return result;
}

// 2D version
inline mat2x3 SetRotation(const mat2x3 &m, const mat2x2 &rot2x2)
{
    mat2x3 result = m;
    for (size_t i = 0; i < 2; ++i) {
        vec2 col = rot2x2[i]; // each column is vec2
        result[i] = col;
    }
    return result;
}

// ------------------------------------------------------------
// Translation: T += offset.x * col0 + offset.y * col1 + offset.z * col2
inline mat3x4 Translate(const mat3x4 &m, const vec3 &offset)
{
    mat3x4 result = m;

    if (offset.x != 0.0) {
        result[3] = result[3] + m[0] * offset[0];
    }
    if (offset.y != 0.0) {
        result[3] = result[3] + m[1] * offset[1];
    }
    if (offset.z != 0.0) {
        result[3] = result[3] + m[2] * offset[2];
    }

    return result;
}

// 2D version
inline mat2x3 Translate(const mat2x3 &m, const vec2 &offset)
{
    mat2x3 result = m;

    if (offset.x != 0.0) {
        result[2] = result[2] + m[0] * offset[0];
    }
    if (offset.y != 0.0) {
        result[2] = result[2] + m[1] * offset[1];
    }
    return result;
}

// ------------------------------------------------------------
// Set absolute translation
inline mat3x4 SetTranslation(const mat3x4 &m, const vec3 &translation)
{
    mat3x4 result = m;
    result[3] = translation;
    return result;
}

// 2D version
inline mat2x3 SetTranslation(const mat2x3 &m, const vec2 &translation)
{
    mat2x3 result = m;
    result[2] = translation;
    return result;
}

// ------------------------------------------------------------
// Multiply two 4x3 transforms (with normalization of the first 3 columns).
inline mat3x4 Transform(const mat3x4 &a, const mat3x4 &b)
{
    mat3x4 result; // 3 rows, 4 columns

    for (size_t col = 0; col < 4; ++col)
    {
        for (size_t row = 0; row < 3; ++row)
        {
            double sum = a[row][0]*b[0][col]
                       + a[row][1]*b[1][col]
                       + a[row][2]*b[2][col];
            if (col == 3) {
                sum += a[row][3];
            }
            result[row][col] = sum;
        }
    }

    // Normalize each of the first 3 columns
    for (size_t i = 0; i < 3; ++i) {
        auto c = result[i];
        auto norm_c = linalg::normalize(c);
        result[i] = norm_c;
    }

    return result;
}

// Invert a 4x3 transform (rotation + translation)
inline mat3x4 InvertTransform(const mat3x4 &m)
{
    // Extract rotation part as a 3x3
    mat3x3 rot;
    for (size_t i = 0; i < 3; ++i) {
        vec3 col = m[i];
        rot[i] = col;
    }

    // Transpose to invert rotation
    auto rotT = linalg::transpose(rot);

    // Put transposed rotation back
    mat3x4 unRotated = SetRotation(m, rotT);

    // The translation is column 3 => invert it
    auto trans = m[3];
    auto invTrans = -linalg::mul(rotT, trans);

    return SetTranslation(unRotated, invTrans);
}

// ------------------------------------------------------------
// 2D versions
inline mat2x3 Transform(const mat2x3 &a, const mat2x3 &b)
{
    mat2x3 result;

    for (size_t col = 0; col < 3; ++col)
    {
        for (size_t row = 0; row < 2; ++row)
        {
            double sum = a[row][0]*b[0][col]
                       + a[row][1]*b[1][col];
            if (col == 2) {
                sum += a[row][2];
            }
            result[row][col] = sum;
        }
    }

    // Normalize first 2 columns
    for (size_t i = 0; i < 2; ++i) {
        auto c = result[i];
        auto norm_c = linalg::normalize(c);
        result[i] = norm_c;
    }

    return result;
}

inline mat2x3 InvertTransform(const mat2x3 &m)
{
    // Extract 2x2 rotation
    mat2x2 rot2;
    for (size_t i = 0; i < 2; ++i) {
        auto col = m[i];
        rot2[i] = col;
    }

    // Transpose to invert
    auto rot2T = linalg::transpose(rot2);

    // Rebuild partial matrix
    mat2x3 unRotated = SetRotation(m, rot2T);

    // Invert translation
    auto trans = m[2];
    auto invTrans = -linalg::mul(rot2T, trans);

    return SetTranslation(unRotated, invTrans);
}

// ------------------------------------------------------------
// CombineTransforms variant (like Transform but different math).
inline mat3x4 CombineTransforms(const mat3x4 &a, const mat3x4 &b)
{
    mat3x4 r;

    auto a0 = a[0];
    auto a1 = a[1];
    auto a2 = a[2];
    auto a3 = a[3];

    // For each rotation column i in b, combine with a, then normalize.
    for (size_t i = 0; i < 3; ++i) {
        auto bi = b[i];
        vec3 c = a0*bi.x + a1*bi.y + a2*bi.z;
        c = linalg::normalize(c);
        r[i] = c;
    }

    // Combine translation
    auto b3 = b[3];
    vec3 c3 = a0*b3.x + a1*b3.y + a2*b3.z + a3;
    r[3] = c3;

    return r;
}

inline mat2x3 CombineTransforms(const mat2x3 &a, const mat2x3 &b)
{
    mat2x3 r;

    auto a0 = a[0];
    auto a1 = a[1];
    auto a2 = a[2];

    for (size_t i = 0; i < 2; ++i) {
        auto bi = b[i];
        vec2 c = a0*bi.x + a1*bi.y;
        c = linalg::normalize(c);
        r[i] = c;
    }

    auto b2 = b[2];
    vec2 c2 = a0*b2.x + a1*b2.y + a2;
    r[2] = c2;

    return r;
}

} // namespace MatrixTransforms
