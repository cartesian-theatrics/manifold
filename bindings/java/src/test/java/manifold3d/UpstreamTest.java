package manifold3d;

import org.junit.Test;
import org.junit.Rule;
import org.junit.rules.TemporaryFolder;
import static org.junit.Assert.*;
import manifold3d.manifold.MeshIO;
import manifold3d.manifold.ExportOptions;
import manifold3d.linalg.DoubleVec3;

public class UpstreamTest {
    @Rule public TemporaryFolder output = new TemporaryFolder();

    @Test public void executionContextLifecycle() {
        try (ExecutionContext context = new ExecutionContext()) {
            assertFalse(context.cancelled());
            assertEquals(1, context.progress(), 0);
            context.cancel();
            assertTrue(context.cancelled());
        }
    }

    @Test public void objGeometryRoundTrip() {
        try (DoubleVec3 size = new DoubleVec3(2, 3, 4);
             Manifold box = Manifold.Cube(size, false);
             Manifold roundTrip = Upstream.ReadOBJ(Upstream.WriteOBJ(box))) {
            assertEquals(24, roundTrip.volume(), 1e-6);
        }
    }

    @Test public void fileErrorsThrowInReleaseBuilds() {
        Manifold.ensureLoaded();
        String missing = output.getRoot().toPath().resolve("absent/file.glb").toString();
        assertThrows(RuntimeException.class, () -> MeshIO.ImportMesh(missing, false));
        try (DoubleVec3 size = new DoubleVec3(2, 3, 4);
             Manifold box = Manifold.Cube(size, false);
             var mesh = box.getMesh();
             ExportOptions options = new ExportOptions()) {
            assertThrows(RuntimeException.class, () -> MeshIO.ExportMesh(missing, mesh, options));
        }
    }
}
