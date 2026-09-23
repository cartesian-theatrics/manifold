package manifold3d.manifold;

import manifold3d.manifold.MeshGL;
import manifold3d.manifold.ExportOptions;

import manifold3d.LibraryPaths;
import org.bytedeco.javacpp.*;
import org.bytedeco.javacpp.annotation.*;


@Platform(compiler = "cpp17", include = { "manifold/meshIO.h" }, link = { "manifold" })
@Namespace("manifold")
public class MeshIO {
    static { Loader.load(); }

    public native static @ByVal MeshGL ImportMesh(@StdString String filename, @Cast("bool") boolean forceCleanup);
    public native static void ExportMesh(@StdString String filename, @Const @ByRef MeshGL mesh, @Const @ByRef ExportOptions options);
}
