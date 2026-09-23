package manifold3d;
import org.bytedeco.javacpp.*;
import org.bytedeco.javacpp.annotation.*;
@Platform(compiler="cpp17", include="model_utils.hpp", link={"manifold"})
@Namespace("manifold")
public class ModelIO extends Pointer {
    static { Loader.load(); }
    public static native @ByVal Model Texture(@Const @ByRef Model model,
        @Cast("const uint8_t*") byte[] image,@Cast("size_t") long count,@Const @ByRef ModelTexture options);
    public static native @ByVal ByteVector ExportGLB(@Const @ByRef Model model,int tileSize);
}
