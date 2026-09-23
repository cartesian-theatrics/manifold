package manifold3d;

import org.junit.Test;
import org.junit.Rule;
import org.junit.rules.TemporaryFolder;
import manifold3d.Manifold;
import manifold3d.linalg.DoubleMat3x4;
import manifold3d.linalg.DoubleMat3x4Vector;
import manifold3d.manifold.MeshGL;
import manifold3d.linalg.DoubleVec3;
import manifold3d.linalg.DoubleVec2;
import manifold3d.linalg.DoubleVec3Vector;
import manifold3d.MeshUtils;
import manifold3d.manifold.MeshIO;
import manifold3d.manifold.CrossSectionVector;
import manifold3d.manifold.CrossSection;
import manifold3d.manifold.ExportOptions;
import java.nio.DoubleBuffer;

public class ManifoldTest {

    @Rule public TemporaryFolder output = new TemporaryFolder();

    public ManifoldTest() {}

    @Test
    public void testManifold() throws Exception {
        // Existing test code
        MeshGL mesh = new MeshGL();
        Manifold manifold = new Manifold(mesh);

        Manifold sphere = Manifold.Sphere(10.0f, 20);
        Manifold cube = Manifold.Cube(new DoubleVec3(15.0f, 15.0f, 15.0f), false);
        Manifold cylinder = Manifold.Cylinder(3, 30.0f, 30.0f, 0, false).translateX(20).translateY(20).translateZ(-3.0);

        Manifold diff = cube.subtract(sphere);
        Manifold intersection = cube.intersect(sphere);
        Manifold union = cube.add(sphere);

        MeshGL diffMesh = diff.getMesh();
        MeshGL intersectMesh = intersection.getMesh();
        MeshGL unionMesh = union.getMesh();
        ExportOptions opts = new ExportOptions();
        opts.faceted(false);

        MeshIO.ExportMesh(output.newFile("CubeMinusSphere.stl").getAbsolutePath(), diffMesh, opts);
        MeshIO.ExportMesh(output.newFile("CubeIntersectSphere.glb").getAbsolutePath(), intersectMesh, opts);
        MeshIO.ExportMesh(output.newFile("CubeUnionSphere.obj").getAbsolutePath(), unionMesh, opts);

        Manifold hull = cylinder.convexHull(cube.translateZ(100.0));
        MeshGL hullMesh = hull.getMesh();

        MeshIO.ExportMesh(output.newFile("hull.glb").getAbsolutePath(), hullMesh, opts);
        assert hull.volume() > 0.0;

        DoubleMat3x4 frame1 = DoubleMat3x4.IdentityMat().translate(new DoubleVec3(20, 0, 0));
        DoubleMat3x4 frame2 = DoubleMat3x4.IdentityMat()
                .translate(new DoubleVec3(20, 0, 30));
        CrossSection section1 = CrossSection.Square(new DoubleVec2(20, 20), true);
        CrossSection section2 = CrossSection.Circle(15, 20);
        Manifold loft = MeshUtils.Loft(new CrossSectionVector(section1, section2),
                                       new DoubleMat3x4Vector(frame1, frame2),
                                       MeshUtils.LoftAlgorithm.EagerNearestNeighbor);

        assert loft.volume() > 0.0;



        // New test code: Creating a simple texture and exporting a GLB file
        // Define the dimensions of the height map
        int width = 10;
        int height = 10;
        float[] heightMap = new float[width * height];
        //Arrays.fill(heightMap, 1.0); // Flat surface

        double maxHeight = 20;

        // Generate a simple height map (e.g., a sine wave pattern)
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                // Create a sine wave pattern
                double z = Math.sin((double)x / width * 2 * Math.PI) * Math.sin((double)y / height * 2 * Math.PI) * maxHeight;
                heightMap[x + y * width] = (float)z;
            }
        }

        // Create a Manifold from the height map
        Manifold texturedSurface = MeshUtils.CreateSurface(heightMap, 1, width, height, 15.0);

        //System.out.println(texturedSurface.status());
        // Export the Manifold to a GLB file
        MeshGL texturedMesh = texturedSurface.getMesh();
        MeshIO.ExportMesh(output.newFile("TexturedSurface.glb").getAbsolutePath(), texturedMesh, opts);

        // Verify that the Manifold has a positive volume
        assert texturedSurface.volume() > 0.0;
    }
}
