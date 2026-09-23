package manifold3d;
import manifold3d.manifold.MeshGL;
import manifold3d.pub.SmoothnessVector;
import org.bytedeco.javacpp.*;
import org.bytedeco.javacpp.annotation.*;

@Platform(compiler = "cpp17", include = "manifold/manifold.h", link = "manifold")
@Namespace("manifold")
public class ExecutionContext extends Pointer {
  static {
    Manifold.ensureLoaded();
    Loader.load();
  }
  public ExecutionContext() {
    allocate();
  }
  private native void allocate();
  @Name("Cancel") public native void cancel();
  @Name("Cancelled") public native boolean cancelled();
  @Name("Progress") public native double progress();
  @Name("FromMeshGL") public native @ByVal Manifold fromMesh(@Const @ByRef MeshGL mesh);
  @Name("Smooth")
  public native @ByVal Manifold smooth(
      @Const @ByRef MeshGL mesh, @Const @ByRef SmoothnessVector sharp);
}
