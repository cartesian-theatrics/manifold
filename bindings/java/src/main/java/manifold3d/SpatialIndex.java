package manifold3d;
import org.bytedeco.javacpp.*;
import org.bytedeco.javacpp.annotation.*;

@Platform(compiler="cpp17", include="spatial_index.hpp", link={"manifold"})
@Namespace("MeshUtils")
public class SpatialIndex extends Pointer {
    static { manifold3d.Manifold.ensureLoaded(); Loader.load(); }
    public SpatialIndex(@Const @ByRef Manifold solid) { allocate(solid); }
    private native void allocate(@Const @ByRef Manifold solid);
    public native @StdVector double[] RayCast(double ox,double oy,double oz,double dx,double dy,double dz,double limit);
    public native @StdVector double[] ClosestPoint(double x,double y,double z);
    public native int ClassifyPoint(double x,double y,double z,double tolerance);
    public native boolean Overlaps(@Const @ByRef SpatialIndex other,double tolerance);
}
