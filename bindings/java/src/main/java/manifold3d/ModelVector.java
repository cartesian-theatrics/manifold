package manifold3d;

import org.bytedeco.javacpp.*;
import org.bytedeco.javacpp.annotation.*;

/** Owns the vector; get() returns an independent, immutable Model value. */
@Platform(compiler="cpp17", include="manifold/model.h", link={"manifold"})
@Name("std::vector<manifold::Model>")
public class ModelVector extends Pointer {
    static { Loader.load(); }
    public ModelVector() { allocate(); }
    private native void allocate();
    public native @Cast("size_t") long size();
    @Name("at") public native @ByVal Model get(@Cast("size_t") long index);
}
