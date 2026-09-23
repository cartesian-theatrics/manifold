package manifold3d;
import manifold3d.linalg.DoubleVec3;
import org.bytedeco.javacpp.*;
import org.bytedeco.javacpp.annotation.*;

@Platform(compiler = "cpp17", include = "upstream.hpp", link = "manifold")
@Name("clj_manifold")
public class Upstream extends Pointer {
  static {
    Manifold.ensureLoaded();
    Loader.load();
  }
  public static class ScalarField extends FunctionPointer {
    static {
      Manifold.ensureLoaded();
      Loader.load();
    }
    protected ScalarField() {
      allocate();
    }
    private native void allocate();
    public native double call(double x, double y, double z);
  }
  public static native @ByVal Manifold LevelSet(ScalarField field, @ByVal DoubleVec3 min,
      @ByVal DoubleVec3 max, double edgeLength, double level, double tolerance,
      ExecutionContext context);
  public static native @ByVal Manifold ReadOBJ(@StdString String text);
  public static native @StdString String WriteOBJ(@Const @ByRef Manifold manifold);
}
