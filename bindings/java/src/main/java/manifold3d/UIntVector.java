package manifold3d;

import java.nio.IntBuffer;
import java.nio.LongBuffer;
import manifold3d.BufferUtils;

import org.bytedeco.javacpp.*;
import org.bytedeco.javacpp.annotation.*;

@Platform(compiler = "cpp17", include = "<vector>")
@Name("std::vector<uint32_t>")
public class UIntVector extends Pointer {
    static { Loader.load(); }

    public UIntVector(Pointer p) { super(p); }
    public UIntVector() { allocate(); }

    public static UIntVector FromArray(long[] values) {
        UIntVector result = new UIntVector();
        for (long value : values) {
            result.pushBack((int) value);
        }
        return result;
    }

    public static UIntVector FromBuffer(LongBuffer values) {
        UIntVector result = new UIntVector();
        LongBuffer copy = values.duplicate();
        while (copy.hasRemaining()) {
            result.pushBack((int) copy.get());
        }
        return result;
    }

    private native void allocate();

    public native @Cast("size_t") long size();
    public native @Cast("bool") boolean empty();
    public native @Cast("unsigned int*") IntPointer data();
    public native void resize(@Cast("size_t") long n);
    public native void reserve(@Cast("size_t") long n);
    public native @Name("operator[]") long get(@Cast("size_t") long n);
    public native @Name("push_back") void pushBack(@Cast("uint32_t") int value);

    public IntBuffer asIntBuffer() {
        return new IntPointer(this).asBuffer();
    }

    public int[] toIntArray() {
        // Allocate an array of the appropriate size
        int size = (int) size();
        int[] result = new int[size];
        IntPointer ptr = data();
        ptr.get(result);
        return result;
    }

    public long[] toLongArray() {
        // Allocate an array of the appropriate size
        int[] signedIntArray = toIntArray();
        long[] result = new long[signedIntArray.length];
        for (int i = 0; i < signedIntArray.length; i++) {
            result[i] = Integer.toUnsignedLong(signedIntArray[i]);
        }
        return result;
    }
}
