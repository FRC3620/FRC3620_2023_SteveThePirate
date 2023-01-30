import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import org.usfirst.frc3620.misc.FieldCalculations;

public class FieldCalculationsTargetTest {
    /* origin in LL corner. Increasing angle CCW. North (heading 0) points to red alliance wall */
    Translation2d target_01 = new Translation2d(0, 5);

    @Test
    public void test00() {
        /*
         * ok. let's try something. We are facing dead south, and we think target is 2 meters in
         * front of us. we should be at x=2, y=5, right?
         */
        Translation2d robot = test (target_01, 2, 0, 180, new Translation2d(2, 5));
        System.out.println(robot);
    }

    @Test
    public void test01() {
        /*
         * ok. let's try something. We are facing dead south, and we think target is 2 meters in
         * front of us, and 2 meters to the left. we should be at x=2, y=7, right?
         */
        Translation2d robot = test(target_01, 2, 2, 180, new Translation2d(2, 7));
        System.out.println(robot);
        robot = test (target_01, 2, -2, 180, new Translation2d(2, 3));
        System.out.println(robot);
    }

    @Test
    public void test02() {
        /*
         * ok. let's try something. We are facing dead southwest, and we think target is √2 meters in
         * front of us, and √2 meters to the left. we should be at x=2, y=5, right?
         */
        Translation2d robot = test (target_01, Math.sqrt(2), Math.sqrt(2), 135, new Translation2d(2, 5));
        System.out.println(robot);
    }

    @Test
    public void test03() {
        /*
         * ok. let's try something. We are facing dead northeast, and we think target is √2 meters in
         * front of us, and √2 meters to the right. we should be at x=2, y=5, right?
         */
        Translation2d robot = test (target_01, Math.sqrt(2), -Math.sqrt(2), -135, new Translation2d(2, 5));
        System.out.println(robot);
    }

    Translation2d test (Translation2d target, double x, double y, double robotHeadingInDegrees, Translation2d expected) {
        Translation2d rv = FieldCalculations.locateCameraViaTarget(target, x, y, Units.degreesToRadians(robotHeadingInDegrees));
        if (expected != null) {
            Assertions.assertEquals(rv.getX(), expected.getX(), 0.01, "X bad");
            Assertions.assertEquals(rv.getY(), expected.getY(), 0.01, "Y bad");
        }
        return rv;
    }

}
