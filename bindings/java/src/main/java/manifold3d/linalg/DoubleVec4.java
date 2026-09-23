package manifold3d.linalg;

import org.bytedeco.javacpp.*;
import org.bytedeco.javacpp.annotation.*;

@Platform(compiler = "cpp17", include = "manifold/linalg.h")
@Namespace("linalg")
@Name("vec<double, 4>")
public class DoubleVec4 extends DoublePointer {
    static { Loader.load(); }

    public DoubleVec4() { allocate(); }
    private native void allocate();

    public DoubleVec4(double x, double y, double z, double w) { allocate(x, y, z, w); }
    private native void allocate(double x, double y, double z, double w);

    @Name("operator[]")
    public native double get(int i);

    public double x() {
        return get(0);
    }
    public double y() {
        return get(1);
    }

    public double z() {
        return get(2);
    }

    public double w() {
        return get(3);
    }

    public native @Name("operator=") @ByRef DoubleVec4 put(@ByRef DoubleVec4 rhs);
}
