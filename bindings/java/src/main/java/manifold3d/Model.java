package manifold3d;
import org.bytedeco.javacpp.*;
import org.bytedeco.javacpp.annotation.*;
import manifold3d.manifold.MeshGL;
import manifold3d.linalg.*;
import manifold3d.pub.Box;

@Platform(compiler="cpp17", include="model_utils.hpp", linkpath={LibraryPaths.MANIFOLD_LIB_DIR}, link={"manifold"})
@Namespace("manifold")
public class Model extends Pointer {
    static { Loader.load(); }
    public Model() { allocate(); }
    private native void allocate();
    public Model(@Const @ByRef Manifold geometry) { allocate(geometry); }
    private native void allocate(@Const @ByRef Manifold geometry);
    public Model(@Const @ByRef Model other) { allocate(other); }
    private native void allocate(@Const @ByRef Model other);
    @Name("Geometry") public native @ByVal Manifold geometry();
    @Name("GetMeshGL") public native @ByVal MeshGL getMeshGL();
    @Name("GetMeshGL") public native @ByVal MeshGL getMeshGL(int index);
    @Name("Translate") public native @ByVal Model translate(@ByVal DoubleVec3 v);
    @Name("Scale") public native @ByVal Model scale(@ByVal DoubleVec3 v);
    @Name("Rotate") public native @ByVal Model rotate(double x,double y,double z);
    @Name("Mirror") public native @ByVal Model mirror(@ByVal DoubleVec3 v);
    @Name("Transform") public native @ByVal Model transform(@Const @ByRef DoubleMat3x4 v);
    @Name("Refine") public native @ByVal Model refine(int n);
    @Name("RefineToLength") public native @ByVal Model refineToLength(double n);
    @Name("SmoothOut") public native @ByVal Model smoothOut(double angle,double amount);
    @Name("CalculateNormals") public native @ByVal Model calculateNormals(int index,double angle);
    @Name("Color") public native @ByVal Model color(@ByVal DoubleVec4 rgba);
    @Name("Boolean") public native @ByVal Model booleanOp(@Const @ByRef Model other,@Cast("manifold::OpType") int op);
    @Name("Compose") public native @ByVal Model compose(@Const @ByRef Model other);
    @Name("Decompose") public native @ByVal ModelVector decompose();
    @Name("Status") public native @Cast("manifold::Manifold::Error") int status();
    @Name("IsEmpty") public native boolean isEmpty();
    @Name("NumVert") public native @Cast("size_t") long numVert();
    @Name("NumTri") public native @Cast("size_t") long numTri();
    @Name("NumProp") public native @Cast("size_t") long numProp();
    @Name("LayerCount") public native @Cast("size_t") long layerCount();
    @Name("ImageCount") public native @Cast("size_t") long imageCount();
    @Name("SurfaceCount") public native @Cast("size_t") long surfaceCount();
    @Name("Volume") public native double volume();
    @Name("SurfaceArea") public native double surfaceArea();
    @Name("Genus") public native int genus();
    @Name("BoundingBox") public native @ByVal Box boundingBox();
    @Name("SampleColor") public native @ByVal DoubleVec4 sampleColor(@Cast("size_t") long triangle,double b1,double b2);
}
