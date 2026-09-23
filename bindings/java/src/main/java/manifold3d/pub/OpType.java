package manifold3d.pub;

import org.bytedeco.javacpp.*;
import org.bytedeco.javacpp.annotation.*;

import manifold3d.LibraryPaths;

@Platform(compiler = "cpp17", include = {"manifold/manifold.h"}, link = {"manifold"})
@Namespace("manifold")
public class OpType extends IntPointer {
    static { manifold3d.Manifold.ensureLoaded(); Loader.load(); }
    public OpType() { allocate(); }
    private native void allocate();
    public OpType(int value) { this(); put(value); }
    public OpType(Pointer p) { super(p); }

    public static final int Add = 0;
    public static final int Subtract = 1;
    public static final int Intersect = 2;
}
