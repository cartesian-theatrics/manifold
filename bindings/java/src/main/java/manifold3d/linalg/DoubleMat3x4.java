package manifold3d.linalg;

import manifold3d.linalg.DoubleVec3;
import manifold3d.linalg.MatrixTransforms;

import org.bytedeco.javacpp.*;
import org.bytedeco.javacpp.annotation.*;

import java.util.Iterator;
import java.lang.Iterable;
import java.util.NoSuchElementException;

@Platform(compiler = "cpp17", include = "manifold/linalg.h")
@Namespace("linalg")
@Name("mat<double, 3, 4>")
public class DoubleMat3x4 extends DoublePointer implements Iterable<DoubleVec3> {
    static { Loader.load(); }

    @Override
    public Iterator<DoubleVec3> iterator() {
        return new Iterator<DoubleVec3>() {

            private int index = 0;

            @Override
            public boolean hasNext() {
                return index < 4;
            }

            @Override
            public DoubleVec3 next() {
                if (!hasNext()) {
                    throw new NoSuchElementException();
                }
                return getColumn(index++);
            }
        };
    }

    public DoubleMat3x4() { allocate(); }
    private native void allocate();

    public static DoubleMat3x4 IdentityMat() {
        return new DoubleMat3x4(new DoubleVec3(1.0, 0.0, 0.0),
                                new DoubleVec3(0.0, 1.0, 0.0),
                                new DoubleVec3(0.0, 0.0, 1.0),
                                new DoubleVec3(0.0, 0.0, 0.0));
    }

    private native void allocate(double x);
    public DoubleMat3x4(double x) { allocate(x); }

    public DoubleMat3x4(@ByRef DoubleVec3 col1, @ByRef DoubleVec3 col2,
                        @ByRef DoubleVec3 col3, @ByRef DoubleVec3 col4) {
        allocate(col1, col2, col3, col4);
    }
    public native void allocate(@ByRef DoubleVec3 col1, @ByRef DoubleVec3 col2,
                                @ByRef DoubleVec3 col3, @ByRef DoubleVec3 col4);

    public DoubleMat3x4(double c0, double c1, double c2, double c3,
                  double c4, double c5, double c6, double c7,
                  double c8, double c9, double c10, double c11) {
        allocate(new DoubleVec3(c0, c1, c2),
                 new DoubleVec3(c3, c4, c5),
                 new DoubleVec3(c6, c7, c8),
                 new DoubleVec3(c9, c10, c11));
    }

    @Name("operator[]") public native @ByRef DoubleVec3 getColumn(int i);

    public native @Name("operator=") @ByRef DoubleMat3x4 put(@ByRef DoubleMat3x4 rhs);

    public DoubleMat3x4 mul(double scalar) {
        return new DoubleMat3x4(
            getColumn(0).x() * scalar, getColumn(0).y() * scalar, getColumn(0).z() * scalar,
            getColumn(1).x() * scalar, getColumn(1).y() * scalar, getColumn(1).z() * scalar,
            getColumn(2).x() * scalar, getColumn(2).y() * scalar, getColumn(2).z() * scalar,
            getColumn(3).x() * scalar, getColumn(3).y() * scalar, getColumn(3).z() * scalar);
    }

    public DoubleMat3x4 transform(@ByRef DoubleMat3x4 other) {
        return MatrixTransforms.Transform(this, other);
    }

    public DoubleMat3x4 rotate(@ByRef DoubleVec3 angles) {
        return MatrixTransforms.Rotate(this, angles);
    }

    public DoubleMat3x4 rotate(@ByRef DoubleVec3 axis, float angle) {
        return MatrixTransforms.Rotate(this, axis, angle);
    }

    public DoubleMat3x4 translate(@ByRef DoubleVec3 vec) {
        return MatrixTransforms.Translate(this, vec);
    }
    public DoubleMat3x4 translate(double x, double y, double z) {
        return MatrixTransforms.Translate(this, new DoubleVec3(x, y, z));
    }
}
