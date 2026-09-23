package manifold3d.manifold;

import org.bytedeco.javacpp.*;
import org.bytedeco.javacpp.annotation.*;

import manifold3d.LibraryPaths;
import manifold3d.linalg.DoubleVec2;
import manifold3d.linalg.DoubleMat2x3;
import manifold3d.manifold.Rect;
import manifold3d.manifold.CrossSectionVector;

import manifold3d.pub.SimplePolygon;
import manifold3d.pub.Polygons;

@Platform(compiler = "cpp17",
          include = { "manifold/cross_section.h" },
          link = { "manifold" })
@Namespace("manifold")
public class CrossSection extends Pointer {
    static { Loader.load(); }

    public CrossSection() { allocate(); }
    private native void allocate();

    public CrossSection(@Const @ByRef CrossSection other) { allocate(other); }
    private native void allocate(@Const @ByRef CrossSection other);

    @Name("operator=")
    public native @ByRef CrossSection put(@Const @ByRef CrossSection other);

    public enum FillRule {
        EvenOdd,   ///< Only odd numbered sub-regions are filled.
        NonZero,   ///< Only non-zero sub-regions are filled.
        Positive,  ///< Only sub-regions with winding counts > 0 are filled.
        Negative   ///< Only sub-regions with winding counts < 0 are filled.
    };

    public enum JoinType {
        Square,
        Round,
        Miter,
        Bevel
    };

    public CrossSection(@Const @ByRef SimplePolygon contour, @Cast("manifold::CrossSection::FillRule") int fillrule) { allocate(contour, fillrule); }
    private native void allocate(@Const @ByRef SimplePolygon contour, @Cast("manifold::CrossSection::FillRule") int fillrule);

    public CrossSection(@ByRef Polygons contours, @Cast("manifold::CrossSection::FillRule") int fillrule) { allocate(contours, fillrule); }
    private native void allocate(@ByRef Polygons contours, @Cast("manifold::CrossSection::FillRule") int fillrule);

    public CrossSection(@ByRef Rect rect) { allocate(rect); }
    private native void allocate(@ByRef Rect rect);

    @Name("Area") public native double area();
    @Name("NumVert") public native int numVert();
    @Name("NumContour") public native int numContour();
    @Name("IsEmpty") public native boolean isEmpty();
    @Name("Bounds") public native @ByVal Rect bounds();

    @Name("Translate") public native @ByVal CrossSection translate(@ByVal DoubleVec2 v);
    public CrossSection translate(double x, double y) {
        return this.translate(new DoubleVec2(x, y));
    }
    public CrossSection translateX(double x) {
        return this.translate(new DoubleVec2(x, 0));
    }
    public CrossSection translateY(double y) {
        return this.translate(new DoubleVec2(0, y));
    }

    @Name("Rotate") public native @ByVal CrossSection rotate(float degrees);
    @Name("Scale") public native @ByVal CrossSection scale(@ByVal DoubleVec2 s);
    @Name("Mirror") public native @ByVal CrossSection mirror(@ByVal DoubleVec2 ax);
    @Name("Transform") public native @ByVal CrossSection transform(@ByVal DoubleMat2x3 m);
    @Name("Simplify") public native @ByVal CrossSection simplify(double epsilon);

    @Name("Offset") public native @ByVal CrossSection offset(double delta, @Cast("manifold::CrossSection::JoinType") int joinType, double miterLimit, int arcTolerance);

    @Name("Boolean") public native @ByVal CrossSection booleanOp(@ByRef CrossSection second, @Cast("manifold::OpType") int op);
    public static native @ByVal CrossSection BatchBoolean(@ByRef CrossSectionVector sections, @Cast("manifold::OpType") int op);

    @Name("Hull") public native @ByVal CrossSection convexHull();
    @Name("Hull") public static native @ByVal CrossSection ConvexHull(@ByRef SimplePolygon pts);
    @Name("Hull") public static native @ByVal CrossSection ConvexHull(@ByRef Polygons pts);
    @Name("Hull") public static native @ByVal CrossSection ConvexHull(@ByRef CrossSectionVector sections);

    public @ByVal CrossSection convexHull(@ByRef CrossSection other) {
        return CrossSection.ConvexHull(new CrossSectionVector(new CrossSection[]{this, other}));
    }

    @Name("operator+") public native @ByVal CrossSection add(@ByRef CrossSection rhs);
    @Name("operator+=") public native @ByVal CrossSection addPut(@ByRef CrossSection rhs);
    @Name("operator-") public native @ByVal CrossSection subtract(@ByRef CrossSection rhs);
    @Name("operator-=") public native @ByRef CrossSection subtractPut(@ByRef CrossSection rhs);
    @Name("operator^") public native @ByVal CrossSection intersect(@ByRef CrossSection rhs);
    @Name("operator^=") public native @ByVal CrossSection intersectPut(@ByRef CrossSection rhs);

    public static native @ByVal CrossSection Compose(@ByRef CrossSectionVector crossSection);
    @Name("Decompose") public native @ByVal CrossSectionVector decompose();

    public static native @ByVal CrossSection Text(@ByRef @StdString String fontFile, @ByRef @StdString String text, int pixelHeight, @Cast("u_int32_t") int interpRes, @Cast("manifold::CrossSection::FillRule") int fillRule);
    public static native @ByVal CrossSection Circle(float radius, int circularSegments);
    public static native @ByVal CrossSection Square(@ByRef DoubleVec2 size, boolean center);
    public static CrossSection Square(double x, double y, boolean center) {
        return CrossSection.Square(new DoubleVec2(x, y), center);
    }
    public static CrossSection Square(double x, double y) {
        return CrossSection.Square(new DoubleVec2(x, y), false);
    }

    @Name("ToPolygons") public native @ByVal Polygons toPolygons();
}
