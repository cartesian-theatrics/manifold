package manifold3d;

import org.junit.Test;
import static org.junit.Assert.*;
import manifold3d.manifold.MeshGL;

public class MeshGLTest {
    @Test public void copiesVertexProperties() {
        Manifold.ensureLoaded();
        try (MeshGL mesh = new MeshGL(); FloatVector values = new FloatVector()) {
            for (float v : new float[]{3, 4, 5, 6, 7, 8}) values.pushBack(v);
            mesh.numProp(3).vertProperties(values);
            values.pushBack(9); // The setter copies; the mesh remains two vertices.
            assertEquals(2, mesh.NumVert());
            assertEquals(7, mesh.vertProperties().get(4), 0);
        }
    }
}
