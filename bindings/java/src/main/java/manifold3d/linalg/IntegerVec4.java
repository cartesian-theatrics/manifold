package manifold3d.linalg;

import org.bytedeco.javacpp.*;
import org.bytedeco.javacpp.annotation.*;

@Platform(compiler = "cpp17", include = "manifold/linalg.h")
@Namespace("linalg")
@Name("vec<int, 4>")
public class IntegerVec4 extends IntPointer {
    static { Loader.load(); }

    public IntegerVec4() { allocate(); }
    private native void allocate();

    public IntegerVec4(int x, int y, int z, int w) { allocate(x, y, z, w); }
    private native void allocate(int x, int y, int z, int w);

    @Name("operator[]")
    public native int get(int i);

    public int x() {
        return get(0);
    }
    public int y() {
        return get(1);
    }
    public int z() {
        return get(2);
    }
    public int w() {
        return get(3);
    }

    public native @Name("operator=") @ByRef IntegerVec4 put(@ByRef IntegerVec4 rhs);
}
