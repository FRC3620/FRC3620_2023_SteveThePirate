import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import org.usfirst.frc3620.misc.FieldCalculations;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class FieldCalculationsIntersectionTest {

    /* origin in LL corner. Increasing angle CCW. Heading 0 points right (high school trig coords) */
    Translation2d t00 = new Translation2d(0, 0);
    Translation2d t01 = new Translation2d(0, 1);
    Translation2d t10 = new Translation2d(1, 0);

    @Test
    public void test00() {
        /* we are at (1, 1) */
        Translation2d rv = triangulate(t10, Units.degreesToRadians(-90), t01, Units.degreesToRadians(180), new Translation2d(1, 1));
        System.out.println (rv);

        /* kaboom, both points on same line */
        rv = triangulate(t00, Units.degreesToRadians(-90), t10, Units.degreesToRadians(-90), null);
        System.out.println (rv);

        /* we are at (0.5, 0.5) */
        rv = triangulate(t00, Units.degreesToRadians(-135), t10, Units.degreesToRadians(-45), new Translation2d(0.5, 0.5));
        System.out.println (rv);
        rv = triangulate(t00, Units.degreesToRadians(-135), t01, Units.degreesToRadians(135), new Translation2d(0.5, 0.5));
        System.out.println (rv);

        for (double delta = -10; delta <= 10; delta += 1.0) {
            /* each individual one of these should have x equals y; the zero case will be woka woka (points 180 degrees from each other) */
            rv = triangulate(t01, Units.degreesToRadians(135 + delta), t10, Units.degreesToRadians(-45 - delta), null);
            System.out.println (delta + " " + rv);
        }

        /* we are at (0.0, 0.0), test case of PI/2 */
        rv = triangulateGivenTan(t01, Double.POSITIVE_INFINITY, t10, 0, new Translation2d(0, 0));
        System.out.println (rv);
        rv = triangulateGivenTan(t10, 0, t01, Double.POSITIVE_INFINITY, new Translation2d(0, 0));
        System.out.println (rv);

        /* we are at (1.0, 1.0), test case of -PI/2 */
        rv = triangulateGivenTan(t01, 0, t10, Double.NEGATIVE_INFINITY, new Translation2d(1, 1));
        System.out.println (rv);
        rv = triangulateGivenTan(t10, Double.NEGATIVE_INFINITY, t01, 0, new Translation2d(1, 1));
        System.out.println (rv);

        /* we are at (0, 2), test case of both -PI/2, should kaboom */
        rv = triangulate(t00, - Math.PI/2, t01, -Math.PI / 2.0, null);
        Assertions.assertEquals (null, rv, "(0,2) case 1 failed");
        System.out.println (rv);
        rv = triangulate(t01, - Math.PI/2, t00, -Math.PI / 2.0, null);
        Assertions.assertEquals (null, rv, "(0,2) case 2 failed");
        System.out.println (rv);

        /* we are at (-1, 0), test case of both 0 */
        rv = triangulate(t00, 0, t10, 0, null);
        Assertions.assertEquals (null, rv, "(0,2) case 1 failed");
        System.out.println (rv);
        rv = triangulate(t10, 0, t00, 0, null);
        Assertions.assertEquals (null, rv, "(0,2) case 2 failed");
        System.out.println (rv);
    }

    @Test
    public void infMath() {
        Assertions.assertTrue(Double.POSITIVE_INFINITY > 10000, "positive infinity isn't larger than 10000");
        Assertions.assertTrue(Double.isInfinite(Double.POSITIVE_INFINITY), "positive infinity isn't infinite");
        Assertions.assertTrue(Double.isInfinite(Double.NEGATIVE_INFINITY),"negative infinity isn't infinite");
    }

    Translation2d triangulate (Translation2d a, double heading_a, Translation2d b, double heading_b, Translation2d expected) {
        var rv = FieldCalculations.locateViaTriangulation(a, heading_a, b, heading_b);
        if (expected != null) {
            try {
                Assertions.assertEquals(rv.getX(), expected.getX(), 0.01, "X bad");
                Assertions.assertEquals(rv.getY(), expected.getY(), 0.01, "Y bad");
            } catch (AssertionError e) {
                System.err.println ("Bad result: " + rv + ", expected " + expected);
                throw e;
            }
        }
        return rv;
    }

    Translation2d triangulateGivenTan (Translation2d a, double tan_a, Translation2d b, double tan_b, Translation2d expected) {
        var rv = FieldCalculations.locateViaTriangulationGivenTan(a, tan_a, b, tan_b);
        if (expected != null) {
            try {
                Assertions.assertEquals(rv.getX(), expected.getX(), 0.01, "X bad");
                Assertions.assertEquals(rv.getY(), expected.getY(), 0.01, "Y bad");
            } catch (AssertionError e) {
                System.err.println ("Bad result: " + rv + ", expected " + expected);
                throw e;
            }
        }
        return rv;
    }

}
