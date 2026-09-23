package manifold3d;


import java.nio.FloatBuffer;
import java.nio.ByteOrder;

import manifold3d.BufferUtils;

import org.bytedeco.javacpp.*;
import org.bytedeco.javacpp.annotation.*;

@Platform(compiler = "cpp17", include = "<vector>")
@Name("std::vector<float>")
public class FloatVector extends Pointer {
    static { Loader.load(); }


    /** Pointer cast constructor. Invokes {@link Pointer#Pointer(Pointer)}. */
    public FloatVector(FloatPointer p) { super(p); }
    public FloatVector() { allocate(); }
    public static FloatVector FromArray(float[] floatArray) {
        try (FloatPointer floatPtr = new FloatPointer(floatArray)) {
            return BufferUtils.floatVectorFromPointer(floatPtr, floatArray.length);
        }
    }
    public static FloatVector FromBuffer(FloatBuffer floatBuffer) {
        // A float buffer is not a std::vector object. Copy its remaining values.
        if (!floatBuffer.hasRemaining()) return new FloatVector();
        if (!floatBuffer.isDirect() || floatBuffer.order() != ByteOrder.nativeOrder()) {
            float[] values = new float[floatBuffer.remaining()];
            floatBuffer.duplicate().get(values);
            return FromArray(values);
        }
        // Slice to position zero: JavaCPP's direct-buffer address must start at
        // the first selected float, not at a byte offset equal to its position.
        try (FloatPointer floatPtr = new FloatPointer(floatBuffer.slice())) {
            return BufferUtils.floatVectorFromPointer(floatPtr, floatBuffer.remaining());
        }
    }
    private native void allocate();
    public native FloatPointer data();
    public native @Cast("size_t") long size();
    public native @Cast("bool") boolean empty();
    public native void resize(@Cast("size_t") long n);
    public native void reserve(@Cast("size_t") long n);
    public native @Name("operator[]") float get(@Cast("size_t") long n);
    public native @Name("push_back") void pushBack(float value);

    public FloatBuffer asFloatBuffer() {
        return new FloatPointer(this).asBuffer();
    }

    public float[] toFloatArray() {
        // Allocate an array of the appropriate size
        int size = (int) size();
        float[] result = new float[size];
        if (size == 0) return result;
        FloatPointer ptr = data();
        ptr.get(result);
        return result;
    }
}
