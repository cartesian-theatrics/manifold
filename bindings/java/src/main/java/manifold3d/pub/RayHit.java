package manifold3d.pub;
import manifold3d.linalg.DoubleVec3;
import org.bytedeco.javacpp.*;
import org.bytedeco.javacpp.annotation.*;
@Platform(compiler = "cpp17", include = "manifold/manifold.h")
@Namespace("manifold")
public class RayHit extends Pointer {
  static {
    Loader.load();
  }
  @MemberGetter public native @Cast("uint64_t") long faceID();
  @MemberGetter public native double distance();
  @Name("position") @MemberGetter public native @ByRef DoubleVec3 point();
  @MemberGetter public native @ByRef DoubleVec3 normal();
}
