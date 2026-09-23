package manifold3d;
import org.bytedeco.javacpp.*;
import org.bytedeco.javacpp.annotation.*;
import manifold3d.linalg.DoubleVec3;
@Platform(compiler="cpp17", include="model_utils.hpp", link={"manifold"})
@Namespace("manifold")
public class ModelTexture extends Pointer {
    static { manifold3d.Manifold.ensureLoaded(); Loader.load(); }
    public ModelTexture() { allocate(); }
    private native void allocate();
    @MemberSetter public native void mapping(@StdString String value);
    @MemberSetter public native void name(@StdString String value);
    @MemberSetter public native void origin(@ByVal DoubleVec3 value);
    @MemberSetter public native void normal(@ByVal DoubleVec3 value);
    @MemberSetter public native void right(@ByVal DoubleVec3 value);
    @MemberSetter public native void width(double value); @MemberSetter public native void height(double value);
    @MemberSetter public native void pixelSize(double value); @MemberSetter public native void opacity(double value);
    @MemberSetter public native void u0(double value); @MemberSetter public native void v0(double value);
    @MemberSetter public native void u1(double value); @MemberSetter public native void v1(double value);
    @MemberSetter public native void axisU(int value); @MemberSetter public native void axisV(int value);
    @MemberSetter public native void scaleU(double value); @MemberSetter public native void scaleV(double value);
    @MemberSetter public native void offsetU(double value); @MemberSetter public native void offsetV(double value);
    @MemberSetter public native void seamAngle(double value);
    @MemberSetter public native void padding(double value);
    @MemberSetter public native void pack(boolean value);
    @MemberSetter public native void depthScale(double value); @MemberSetter public native void depthOffset(double value);
    @MemberSetter public native void depthFade(double value); @MemberSetter public native void step(boolean value);
    public native void SetDepth(@Const double[] values,@Cast("size_t") long count,int width,int height);
    public native void SetDepthImage(@Cast("const uint8_t*") byte[] bytes,@Cast("size_t") long count);
}
