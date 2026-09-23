package manifold3d.manifold;

import manifold3d.LibraryPaths;
import org.bytedeco.javacpp.*;
import org.bytedeco.javacpp.annotation.*;

import manifold3d.UIntVector;
import manifold3d.ByteVector;
import manifold3d.FloatVector;

@Platform(compiler = "cpp17", include = "manifold/meshIO.h", link = { "manifold" })
@Namespace("manifold")
public class MeshGL extends Pointer {
    static { manifold3d.Manifold.ensureLoaded(); Loader.load(); }

    public MeshGL() { allocate(); }
    private native void allocate();
    public MeshGL(@Const @ByRef MeshGL other) { allocate(other); }
    private native void allocate(@Const @ByRef MeshGL other);

    public native @Cast("uint32_t") int NumVert();
    public native @Cast("uint32_t") int NumTri();
    public native @Cast("uint32_t") int NumRun();
    public native @ByRef ByteVector runFlags();
    public native MeshGL runFlags(@ByRef ByteVector flags);
    public native boolean HasNormals(@Cast("size_t") long run);
    public native boolean Backside(@Cast("size_t") long run);
    public native float tolerance();
    public native MeshGL tolerance(float value);

    public native @Cast("uint32_t") int numProp();
    public native MeshGL numProp(@Cast("uint32_t") int numProp);

    public native @ByRef FloatVector vertProperties();
    public native MeshGL vertProperties(@ByRef FloatVector vertProperties);

    public native @ByRef UIntVector triVerts();
    public native MeshGL triVerts(@ByRef UIntVector triVerts);

    public native @ByRef UIntVector mergeFromVert();
    public native MeshGL mergeFromVert(@ByRef UIntVector mergeFromVert);

    public native @ByRef UIntVector mergeToVert();
    public native MeshGL mergeToVert(@ByRef UIntVector mergeToVert);

    public native @ByRef UIntVector runIndex();
    public native MeshGL runIndex(@ByRef UIntVector runIndex);

    public native @ByRef UIntVector runOriginalID();
    public native MeshGL runOriginalID(@ByRef UIntVector runOriginalID);

    public native @ByRef FloatVector runTransform();
    public native MeshGL runTransform(@ByRef FloatVector runTransform);

    public native @ByRef UIntVector faceID();
    public native MeshGL faceID(@ByRef UIntVector faceID);

    public native @ByRef FloatVector halfedgeTangent();
    public native MeshGL halfedgeTangent(@ByRef FloatVector halfedgeTangent);

    // MeshGL constructor and other methods
    public native boolean Merge();
}
