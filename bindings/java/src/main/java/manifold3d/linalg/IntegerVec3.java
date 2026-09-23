package manifold3d.linalg;

import org.bytedeco.javacpp.*;
import org.bytedeco.javacpp.annotation.*;

@Platform(compiler = "cpp17", include = "manifold/linalg.h")
@Namespace("linalg")
@Name("vec<int, 3>")
public class IntegerVec3 extends IntPointer {
    static { Loader.load(); }

    public IntegerVec3() { allocate(); }
    private native void allocate();

    public IntegerVec3(int x, int y, int z) { allocate(x, y, z); }
    private native void allocate(int x, int y, int z);

    @Name("operator []")
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

    //public native void put(int i, int value);
    //public native void set(int component, int value);

    public native @Name("operator=") @ByRef IntegerVec3 put(@ByRef IntegerVec3 rhs);
}
