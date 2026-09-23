package manifold3d;
import org.bytedeco.javacpp.*;
import org.bytedeco.javacpp.annotation.*;
@Platform(compiler="cpp17", include="vector")
@Name("std::vector<uint8_t>")
public class ByteVector extends Pointer {
    static { Loader.load(); }
    public ByteVector() { allocate(); }
    private native void allocate();
    public native @Cast("size_t") long size();
    @Index public native @Cast("uint8_t") byte get(@Cast("size_t") long index);
    public native @Cast("uint8_t*") BytePointer data();
    public byte[] toByteArray() {
        byte[] result = new byte[Math.toIntExact(size())];
        if (result.length > 0) data().get(result);
        return result;
    }
}
