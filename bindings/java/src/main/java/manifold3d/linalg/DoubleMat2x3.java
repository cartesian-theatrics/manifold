package manifold3d.linalg;

import manifold3d.linalg.DoubleVec2;

import org.bytedeco.javacpp.*;
import org.bytedeco.javacpp.annotation.*;

import java.util.Iterator;
import java.lang.Iterable;
import java.util.NoSuchElementException;

@Platform(compiler = "cpp17", include = "linalg.h")
@Namespace("linalg")
@Name("mat<double, 2, 3>")
public class DoubleMat2x3 extends DoublePointer implements Iterable<DoubleVec2> {
    static { Loader.load(); }

    @Override
    public Iterator<DoubleVec2> iterator() {
        return new Iterator<DoubleVec2>() {

            private int index = 0;

            @Override
            public boolean hasNext() {
                return index < 3;
            }

            @Override
            public DoubleVec2 next() {
                if (!hasNext()) {
                    throw new NoSuchElementException();
                }
                return getColumn(index++);
            }
        };
    }

    public DoubleMat2x3() { allocate(); }
    private native void allocate();

    public static DoubleMat2x3 IdentityMat() {
        return new DoubleMat2x3(new DoubleVec2(1.0, 0.0),
                                new DoubleVec2(0.0, 1.0),
                                new DoubleVec2(0.0, 0.0));
    }

    public DoubleMat2x3(double x) { allocate(x); }
    private native void allocate(double x);

    public DoubleMat2x3(@ByRef DoubleVec2 col1, @ByRef DoubleVec2 col2, @ByRef DoubleVec2 col3) {
        allocate(col1, col2, col3);
    }
    public native void allocate(@ByRef DoubleVec2 col1, @ByRef DoubleVec2 col2, @ByRef DoubleVec2 col3);

    public DoubleMat2x3(double c0, double c1, double c2,
                        double c3, double c4, double c5) {
        allocate(new DoubleVec2(c0, c1),
                 new DoubleVec2(c2, c3),
                 new DoubleVec2(c4, c5));
    }

    @Name("operator[]") public native @ByRef DoubleVec2 getColumn(int i);

    public native @Name("operator=") @ByRef DoubleMat2x3 put(@ByRef DoubleMat2x3 rhs);
}
