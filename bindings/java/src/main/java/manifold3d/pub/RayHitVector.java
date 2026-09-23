package manifold3d.pub;
import org.bytedeco.javacpp.*;
import org.bytedeco.javacpp.annotation.*;
@Platform(compiler = "cpp17", include = {"<vector>", "manifold/manifold.h"})
@Name("std::vector<manifold::RayHit>")
public class RayHitVector extends Pointer {
  static {
    Loader.load();
  }
  public native @Cast("size_t") long size();
  @Name("operator[]") public native @ByRef RayHit get(@Cast("size_t") long index);
}
