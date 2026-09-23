package manifold3d;

import org.junit.Test;
import org.junit.Rule;
import org.junit.rules.TemporaryFolder;
import static org.junit.Assert.*;
import manifold3d.Manifold;
import manifold3d.linalg.DoubleVec2;
import manifold3d.manifold.MeshGL;
import manifold3d.pub.SimplePolygon;
import manifold3d.pub.Polygons;
import manifold3d.manifold.MeshIO;
import manifold3d.manifold.ExportOptions;
import manifold3d.manifold.CrossSection;
import manifold3d.manifold.CrossSection.FillRule;
import manifold3d.manifold.MeshIO;
import manifold3d.manifold.ExportOptions;
import manifold3d.Manifold;

public class CrossSectionTest {

    @Rule public TemporaryFolder output = new TemporaryFolder();

    public CrossSectionTest() {}

    @Test
    public void testCrossSection() throws Exception {
        SimplePolygon polygon = new SimplePolygon();
        polygon.pushBack(new DoubleVec2(-10.0, -10.0));
        polygon.pushBack(new DoubleVec2(10.0, -10.0));
        polygon.pushBack(new DoubleVec2(10.0, 10.0));
        polygon.pushBack(new DoubleVec2(-10.0, 10.0));

        SimplePolygon innerPolygon = new SimplePolygon();
        innerPolygon.pushBack(new DoubleVec2(-5.0, -5.0));
        innerPolygon.pushBack(new DoubleVec2(5.0, -5.0));
        innerPolygon.pushBack(new DoubleVec2(5.0, 5.0));
        innerPolygon.pushBack(new DoubleVec2(-5.0, 5.0));

        CrossSection section = new CrossSection(polygon, FillRule.NonZero.ordinal());
        CrossSection innerSection = new CrossSection(innerPolygon, FillRule.NonZero.ordinal())
            .translate(new DoubleVec2(3, 0));

        assertEquals(400, section.area(), 0.001);
        assertEquals(100, innerSection.area(), 0.001);

        CrossSection circle = CrossSection.Circle(3.0f, 20);
        Manifold cylinder = Manifold.Extrude(circle.toPolygons(), 50, 60, 0, new DoubleVec2(1.0, 1.0));

        CrossSection unionSection = section.convexHull(CrossSection.Circle(5, 0).translateX(60));

        assert unionSection.area() > 0.0;

        Manifold man = Manifold.Extrude(unionSection.toPolygons(), 50, 60, 0, new DoubleVec2(1.0, 1.0));
        MeshGL mesh = man.getMesh();
        ExportOptions opts = new ExportOptions();
        MeshIO.ExportMesh(output.newFile("CrossSectionTest.stl").getAbsolutePath(), mesh, opts);
    }
}
