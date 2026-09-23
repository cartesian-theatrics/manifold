package manifold3d;

import org.bytedeco.javacpp.*;
import org.bytedeco.javacpp.annotation.*;

import manifold3d.LibraryPaths;
import manifold3d.manifold.CrossSectionVector;
import manifold3d.manifold.CrossSection;
import manifold3d.linalg.DoubleVec4;
import manifold3d.linalg.DoubleVec3Vector;
import manifold3d.linalg.DoubleMat3x4Vector;
import manifold3d.pub.PolygonsVector;
import manifold3d.pub.Polygons;
import manifold3d.pub.SimplePolygon;
import manifold3d.UIntVecVector;

import manifold3d.Manifold;
import java.nio.DoubleBuffer;
import java.nio.FloatBuffer;
import java.nio.IntBuffer;

@Platform(compiler = "cpp17", include = {"mesh_utils.hpp", "buffer_utils.hpp"}, linkpath = { LibraryPaths.MANIFOLD_LIB_DIR }, link = {"manifold"})
public class MeshUtils extends Pointer {
    static { Loader.load(); }

    public MeshUtils() { }

    public static enum LoftAlgorithm {
        EagerNearestNeighbor(0),
        Isomorphic(1);

        public final long value;
        private LoftAlgorithm(long v) { this.value = v; }
    }

    public static native @ByVal Manifold Polyhedron(DoublePointer vertices, @Cast("std::size_t") long nVertices, IntPointer faceBuf, IntPointer faceLengths, @Cast("std::size_t") long nFaces);
    public static Manifold PolyhedronFromBuffers(DoubleBuffer vertices, long nVertices, IntBuffer faceBuf, IntBuffer faceLengths, long nFaces) {

        DoublePointer verticesPtr = new DoublePointer(vertices);
        IntPointer faceBufPtr = new IntPointer(faceBuf);
        IntPointer lengthsPtr = new IntPointer(faceLengths);

        return Polyhedron(verticesPtr, nVertices, faceBufPtr, lengthsPtr, nFaces);
    }

    public static native @ByVal Manifold LoadImage(@Const @StdString String filename, float depth);
    public static native @ByVal Manifold LoadImage(@Const @StdString String filename, float depth, double pixelWidth);

    public static native @ByVal Manifold ColorVertices(@Const @ByRef Manifold manifold, @Const @ByRef DoubleVec4 rgba);
    public static native @ByVal Manifold ColorVertices(@Const @ByRef Manifold manifold, @Const @ByRef DoubleVec4 rgba, long propIndex);
    public static native @ByVal Manifold ApplyPlanarUV(
        @Const @ByRef Manifold manifold,
        @Cast("std::size_t") long propIndex,
        int axisU,
        int axisV,
        double scaleU,
        double scaleV,
        double offsetU,
        double offsetV);
    public static native @ByVal Manifold ApplyBoxUV(
        @Const @ByRef Manifold manifold, @Cast("std::size_t") long propIndex,
        double originX, double originY, double originZ, double sizeU, double sizeV,
        double scaleU, double scaleV, double offsetU, double offsetV);
    public static native @ByVal Manifold UnwrapUV(
        @Const @ByRef Manifold manifold,
        @Cast("std::size_t") long propIndex,
        double seamAngleDegrees,
        double scale,
        double padding,
        boolean pack);
    public static native @ByVal Manifold GeodesicUV(
        @Const @ByRef Manifold manifold,
        @Cast("std::size_t") long propIndex,
        double originX,
        double originY,
        double originZ,
        double normalX,
        double normalY,
        double normalZ,
        double uDirectionX,
        double uDirectionY,
        double uDirectionZ,
        double sizeU,
        double sizeV,
        double atlasU,
        double atlasV,
        double atlasWidth,
        double atlasHeight,
        double outsideU,
        double outsideV,
        double pixelSize);
    /** Signed row-major depth samples, bilinearly sampled in local image UVs. */
    public static native @ByVal Manifold GeodesicUVDepth(
        @Const @ByRef Manifold manifold,
        @Cast("std::size_t") long propIndex,
        double originX, double originY, double originZ,
        double normalX, double normalY, double normalZ,
        double uDirectionX, double uDirectionY, double uDirectionZ,
        double sizeU, double sizeV,
        double atlasU, double atlasV, double atlasWidth, double atlasHeight,
        double outsideU, double outsideV, double pixelSize,
        @StdVector double[] depth, int width, int height,
        double depthScale, double depthOffset, double depthFade, boolean stepBoundary);
    /** Decode an 8/16-bit grayscale depth image and displace entirely in C++. */
    public static native @ByVal Manifold GeodesicUVDepthImage(
        @Const @ByRef Manifold manifold,
        @Cast("std::size_t") long propIndex,
        double originX, double originY, double originZ,
        double normalX, double normalY, double normalZ,
        double uDirectionX, double uDirectionY, double uDirectionZ,
        double sizeU, double sizeV,
        double atlasU, double atlasV, double atlasWidth, double atlasHeight,
        double outsideU, double outsideV, double pixelSize,
        @Const @StdString String filename,
        double depthScale, double depthOffset, double depthFade, boolean stepBoundary);
    public static native @ByVal Manifold CreateSurface(@Const FloatPointer heightMap, int numProps, int width, int height);
    public static native @ByVal Manifold CreateSurface(@Const FloatPointer heightMap, int numProps, int width, int height, double pixelWidth);
    public static native @ByVal Manifold CreateSurface(@Const @StdString String filename);
    public static native @ByVal Manifold CreateSurface(@Const @StdString String filename, double pixelWidth);
    public static Manifold CreateSurface(float[] heightMapArray, int width, int height) {
        FloatPointer heightMapPtr = new FloatPointer(heightMapArray);
        return CreateSurface(heightMapPtr, 1, width, height);
    }
    public static Manifold CreateSurface(float[] heightMapArray, int numProps, int width, int height) {
        FloatPointer heightMapPtr = new FloatPointer(heightMapArray);
        return CreateSurface(heightMapPtr, numProps, width, height);
    }
    public static Manifold CreateSurface(float[] heightMapArray, int numProps, int width, int height, double pixelWidth) {
        FloatPointer heightMapPtr = new FloatPointer(heightMapArray);
        return CreateSurface(heightMapPtr, numProps, width, height, pixelWidth);
    }
    public static Manifold CreateSurface(FloatBuffer heightMapBuffer, int width, int height) {
        FloatPointer heightMapPtr = new FloatPointer(heightMapBuffer);
        return CreateSurface(heightMapPtr, 1, width, height);
    }
    public static Manifold CreateSurface(FloatBuffer heightMapBuffer, int numProps, int width, int height) {
        FloatPointer heightMapPtr = new FloatPointer(heightMapBuffer);
        return CreateSurface(heightMapPtr, numProps, width, height);
    }
    public static Manifold CreateSurface(FloatBuffer heightMapBuffer, int numProps, int width, int height, double pixelWidth) {
        FloatPointer heightMapPtr = new FloatPointer(heightMapBuffer);
        return CreateSurface(heightMapPtr, numProps, width, height, pixelWidth);
    }

    public static native @ByVal Manifold PlyToSurface(@Const @StdString String filepath, double cellSize, double zOffset, double scaleFactor);

    public static native @ByVal Manifold Loft(@ByRef SimplePolygon polygon, @ByRef DoubleMat3x4Vector transforms);
    public static native @ByVal Manifold Loft(@ByRef SimplePolygon polygon, @ByRef DoubleMat3x4Vector transforms, LoftAlgorithm algorithmEnum);
    public static native @ByVal Manifold Loft(@ByRef Polygons polygons, @ByRef DoubleMat3x4Vector transforms);
    public static native @ByVal Manifold Loft(@ByRef Polygons polygons, @ByRef DoubleMat3x4Vector transforms, LoftAlgorithm algorithmEnum);
    public static native @ByVal Manifold Loft(@ByRef PolygonsVector polygons, @ByRef DoubleMat3x4Vector transforms);
    public static native @ByVal Manifold Loft(@ByRef PolygonsVector polygons, @ByRef DoubleMat3x4Vector transforms, LoftAlgorithm algorithmEnum);
    public static native @ByVal Manifold Loft(@ByRef CrossSectionVector sections, @ByRef DoubleMat3x4Vector transforms);
    public static native @ByVal Manifold Loft(@ByRef CrossSectionVector sections, @ByRef DoubleMat3x4Vector transforms, LoftAlgorithm algorithmEnum);
    public static native @ByVal Manifold Loft(@ByRef CrossSection section, @ByRef DoubleMat3x4Vector transforms);
    public static native @ByVal Manifold Loft(@ByRef CrossSection section, @ByRef DoubleMat3x4Vector transforms, LoftAlgorithm algorithmEnum);
}
