package manifold3d;

import manifold3d.linalg.DoubleVec3;

/** Run against the release jar with the native build directory hidden. */
public class PackagedSmoke {
    public static void main(String[] args) {
        // Must work as the first entry point, without relying on a prior cube.
        try (ManifoldVector values = new ManifoldVector();
             ExecutionContext context = new ExecutionContext();
             DoubleVec3 size = new DoubleVec3(2, 3, 4);
             Manifold box = Manifold.Cube(size, false);
             Manifold copy = Upstream.ReadOBJ(Upstream.WriteOBJ(box))) {
            if (values.size() != 0 || context.cancelled() ||
                Math.abs(copy.volume() - 24) > 1e-6) {
                throw new AssertionError("Packaged JNI smoke test failed");
            }
            context.cancel();
            if (!context.cancelled()) throw new AssertionError("Cancellation failed");
            System.out.println("Packaged JNI native loading, OBJ round-trip and context passed");
        }
    }
}
