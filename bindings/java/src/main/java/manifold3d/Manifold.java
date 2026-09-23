package manifold3d;

import java.net.URL;
import java.security.CodeSource;
import java.security.ProtectionDomain;


import manifold3d.UIntVector;
import manifold3d.FloatVector;
import manifold3d.IntVector;

import java.io.IOException;
import java.io.File;

import manifold3d.ManifoldPair;
import manifold3d.ManifoldVector;
import manifold3d.manifold.MeshGL;
import manifold3d.manifold.ExportOptions;

import manifold3d.pub.Box;
import manifold3d.pub.Polygons;
import manifold3d.pub.PolygonsVector;
import manifold3d.pub.SmoothnessVector;
import manifold3d.pub.OpType;

import manifold3d.linalg.DoubleVec3Vector;
import manifold3d.linalg.DoubleMat3x4;
import manifold3d.linalg.DoubleVec2;
import manifold3d.linalg.DoubleVec3;
import manifold3d.linalg.IntegerVec3;

import org.bytedeco.javacpp.*;
import org.bytedeco.javacpp.annotation.*;

@Platform(compiler = "cpp17", include = {"manifold/manifold.h", "manifold/meshIO.h"},
          link = { "manifold" })
@Namespace("manifold")
public class Manifold extends Pointer {
    static {

        String osName = System.getProperty("os.name").toLowerCase();
        if (osName.contains("linux")) {
            try {
                System.load(Loader.extractResource("/libmanifold.so", null, "libmanifold", ".so").getAbsolutePath());
            } catch (IOException e) {
                throw new RuntimeException(e);
            }
        } else if (osName.contains("windows")) {
            try {
                System.out.println("Loading manifold");
                System.load(Loader.extractResource("/manifold.dll", null, "manifold", ".dll").getAbsolutePath());
                System.out.println("Finished Loading.");
            } catch (IOException e) {
                throw new RuntimeException(e);
            }
        } else if (osName.contains("mac")) {
            try {
                System.out.println("Loading Manifold");
                System.load(Loader.extractResource("/libmanifold.dylib", null, "libmanifold", ".dylib").getAbsolutePath());
                System.out.println("Finished Loading.");
            } catch (IOException e) {
                throw new RuntimeException(e);
            }
        } else {
            throw new UnsupportedOperationException("Unsupported operating system: " + osName);
        }

        Loader.load();
    }

    /** Initialize the packaged native library before another JNI entry point. */
    public static void ensureLoaded() {}

    // Constructors and destructor
    public Manifold() { allocate(); }
    private native void allocate();

    public Manifold(@ByRef Manifold other) { allocate(other); }
    private native void allocate(@ByRef Manifold other);

    public native @ByRef @Name("operator=") Manifold put(@ByRef Manifold other);

    public Manifold(@ByRef MeshGL mesh) { allocate(mesh); }
    private native void allocate(@ByRef MeshGL mesh);

    // Methods
    public MeshGL getMesh() {
        return getMeshGL(-1);
    }
    @Name("GetMeshGL") public native @ByVal MeshGL getMeshGL(int normalIdx);
    @Name("GetMeshGL") public native @ByVal MeshGL getMeshGL();
    @Name("IsEmpty") public native boolean isEmpty();
    @Name("Status") public native @Cast("manifold::Manifold::Error") int status();
    @Name("NumVert") public native int numVert();
    @Name("NumEdge") public native int numEdge();
    @Name("NumTri") public native int numTri();
    @Name("NumProp") public native int numProp();
    @Name("NumPropVert") public native int numPropVert();
    @Name("BoundingBox") public native @ByVal Box boundingBox();
    @Name("SurfaceArea") public native double surfaceArea();
    @Name("Volume") public native double volume();
    //@Name("Precision") public native float precision();
    @Name("Genus") public native int genus();
    @Name("CalculateCurvature") public native @ByVal Manifold calculateCurvature(int gaussianIdx, int meanIdx);
    @Name("OriginalID") public native int originalID();
    @Name("AsOriginal") public native @ByVal Manifold asOriginal();

    @Name("Hull") public native @ByVal Manifold convexHull();
    @Name("Hull") public static native @ByVal Manifold ConvexHull(@ByRef ManifoldVector manifolds);

    public @ByVal Manifold convexHull(@ByRef Manifold other) {
        Manifold[] vec  = new Manifold[]{this, other};
        return Manifold.ConvexHull(new ManifoldVector(vec));
    }

    //// Modifiers
    @Name("Translate") public native @ByVal Manifold translate(@ByRef DoubleVec3 translation);
    public Manifold translate(double x, double y, double z) {
        return this.translate(new DoubleVec3(x, y, z));
    }
    public Manifold translateX(double x) {
        return this.translate(new DoubleVec3(x, 0, 0));
    }
    public Manifold translateY(double y) {
        return this.translate(new DoubleVec3(0, y, 0));
    }
    public Manifold translateZ(double z) {
        return this.translate(new DoubleVec3(0, 0, z));
    }

    @Name("Scale") public native @ByVal Manifold scale(@ByRef DoubleVec3 scale);
    public Manifold scale(double x, double y, double z) {
        return this.scale(new DoubleVec3(x, y, z));
    }

    @Name("Rotate") public native @ByVal Manifold rotate(float xDegrees, float yDegrees, float zDegrees);
    @Name("Transform") public native @ByVal Manifold transform(@ByRef DoubleMat3x4 transform);
    @Name("Mirror") public native @ByVal Manifold mirror(@ByRef DoubleVec3 mirrorAxis);
    @Name("Refine") public native @ByVal Manifold refine(int refineValue);

    // CSG operators
    @Name("Boolean") public native @ByVal Manifold booleanOp(@ByRef Manifold second, @Cast("manifold::OpType") int op);
    @Name("operator+") public native @ByVal Manifold add(@ByRef Manifold manifold);
    @Name("operator+=") public native @ByRef Manifold addPut(@ByRef Manifold manifold);
    @Name("operator-") public native @ByVal Manifold subtract(@ByRef Manifold manifold);
    @Name("operator-=") public native @ByRef Manifold subtractPut(@ByRef Manifold manifold);
    @Name("operator^") public native @ByVal Manifold intersect(@ByRef Manifold manifold);
    @Name("operator^=") public native @ByRef Manifold intersectPut(@ByRef Manifold manifold);

    @Name("SplitByPlane")
    public native @ByVal ManifoldPair splitByPlane(@ByRef DoubleVec3 normal, float originOffset);

    @Name("Split")
    public native @ByVal ManifoldPair split(@ByRef Manifold otehr);

    @Name("TrimByPlane")
    public native @ByVal Manifold trimByPlane(@ByRef DoubleVec3 normal, float originOffset);

    @Name("Decompose")
    public native @ByVal ManifoldVector decompose();

    @Name("Slice")
    public native @ByVal Polygons slice(float height);
    public Polygons slice() {
        return this.slice((float) 0.0);
    }

    @Name("Slices")
    public native @ByVal PolygonsVector slices(float bottomZ, float topZ, int nSlices);

    @Name("CalculateNormals")
    public native @ByVal Manifold calculateNormals(int normalIdx, float minSharpAngle);

    @Name("SmoothByNormals")
    public native @ByVal Manifold smoothByNormals(int normalIdx);

    @Name("SmoothOut")
    public native @ByVal Manifold smoothOut(float minSharpAngle, float minSmoothness);

    @Name("Project")
    public native @ByVal Polygons project();

    @Name("RefineToLength")
    public native @ByVal Manifold refineToLength(float length);

    @Name("GetFaceNormals")
    public native @ByVal FloatVector getFaceNormals();
    @Name("GetHalfedges")
    public native @ByVal IntVector getHalfedges();
    @Name("GetVertices")
    public native @ByVal FloatVector getVertices();
    @Name("GetTriangles")
    public native @ByVal IntVector getTriangles();

    @Name("BatchBoolean")
    public static native @ByVal Manifold BatchBoolean(@ByRef ManifoldVector manifolds, @Cast("manifold::OpType") int op);

    //// Static methods
    public static native @ByVal Manifold Smooth(@ByRef MeshGL mesh, @ByRef SmoothnessVector sharpenedEdges);
    public static native @ByVal Manifold Tetrahedron();
    public static native @ByVal Manifold Cube(@ByRef DoubleVec3 size, boolean center);

    public static native @ByVal Manifold Cylinder(float height, float radiusLow, float radiusHigh, int circularSegments, boolean center);
    public static native @ByVal Manifold Cylinder(float height, float radiusLow, float radiusHigh, int circularSegments);
    public static native @ByVal Manifold Cylinder(float height, float radiusLow, float radiusHigh);
    public static native @ByVal Manifold Cylinder(float height, float radius);

    public static native @ByVal Manifold Sphere(float radius, int circularSegments);
    public static native @ByVal Manifold Extrude(@ByRef Polygons crossSection, float height, int nDivisions, float twistDegrees, @ByRef DoubleVec2 scaleTop);

    public static native @ByVal Manifold Revolve(@ByRef Polygons crossSection, int circularSegments);
    public static native @ByVal Manifold Revolve(@ByRef Polygons crossSection, int circularSegments, float revolveDegrees);
    public static native @ByVal Manifold Compose(@ByRef ManifoldVector manifolds);



    @Name("WithContext") public native @ByVal Manifold withContext(@Const @ByRef ExecutionContext context);
    @Name("MinkowskiSum") public native @ByVal Manifold minkowskiSum(@Const @ByRef Manifold other);
    @Name("MinkowskiDifference") public native @ByVal Manifold minkowskiDifference(@Const @ByRef Manifold other);
    @Name("Simplify") public native @ByVal Manifold simplify(double tolerance);
    @Name("RefineToTolerance") public native @ByVal Manifold refineToTolerance(double tolerance);
    @Name("GetTolerance") public native double getTolerance();
    @Name("SetTolerance") public native @ByVal Manifold setTolerance(double tolerance);
    @Name("MinGap") public native double minGap(@Const @ByRef Manifold other, double searchLength);
    @Name("RayCast") public native @ByVal manifold3d.pub.RayHitVector rayCast(@ByVal DoubleVec3 origin, @ByVal DoubleVec3 endpoint);
}
