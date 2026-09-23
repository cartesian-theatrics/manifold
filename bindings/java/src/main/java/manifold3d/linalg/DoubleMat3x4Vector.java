package manifold3d.linalg;

import java.nio.DoubleBuffer;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.DoubleBuffer;

import java.util.Arrays;

import manifold3d.BufferUtils;
import manifold3d.linalg.DoubleMat3x4;
import org.bytedeco.javacpp.*;
import org.bytedeco.javacpp.annotation.*;

import java.util.ArrayList;
import java.util.Iterator;
import java.lang.Iterable;
import java.util.NoSuchElementException;

@Platform(compiler = "cpp17", include = {"<vector>", "manifold/linalg.h"})
@Name("std::vector<linalg::mat<double, 3, 4>>")
public class DoubleMat3x4Vector extends Pointer implements Iterable<DoubleMat3x4> {
    static { Loader.load(); }

    @Override
    public Iterator<DoubleMat3x4> iterator() {
        return new Iterator<DoubleMat3x4>() {

            private long index = 0;

            @Override
            public boolean hasNext() {
                return index < size();
            }

            @Override
            public DoubleMat3x4 next() {
                if (!hasNext()) {
                    throw new NoSuchElementException();
                }
                return get(index++);
            }
        };
    }
    public DoubleMat3x4Vector(ArrayList<DoubleMat3x4> mats) {
        allocate();
        for (DoubleMat3x4 section: mats) {
            this.pushBack(section);
        }
    }

    public DoubleMat3x4Vector(DoubleMat3x4... mats) {
        allocate();
        for (DoubleMat3x4 section : mats) {
            this.pushBack(section);
        }
    }

    public DoubleMat3x4Vector() { allocate(); }
    private native void allocate();

    public native @Cast("size_t") long size();
    public native void resize(@Cast("size_t") long n);

    @Name("operator[]") public native @ByRef DoubleMat3x4 get(@Cast("size_t") long i);
    @Name("push_back") public native void pushBack(@ByRef DoubleMat3x4 value);
}
