import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import org.usfirst.frc3620.misc.FieldCalculations;

public class FieldCalculationsTarget3dTest {
    static final double S2 = Math.sqrt(2.0);
    static final double S3 = Math.sqrt(3.0);

    /*
     *            +Y
     *            ^ 
     *            ╟─────────┐
     *            ║ NW N NE │
     *  Blue Wall ║ W  .  E │ Red Wall
     *            ║ SW S SE │
     *       Y=0 ═╬═════════╧═> +X
     *           X=0  ▄▄▄ 
     *              Scorer's
     *               table 
     *
     * origin in SW corner of field (blue alliance wall).
     * Heading 0 points to red alliance wall. Increasing angle CCW.
     * 
     */

    Translation3d target_051 = new Translation3d(0, 5, 1);

    @Test
    public void test00() {
        /*
         * We are facing the blue wall, level, and we think target_051 is 2 meters in
         * front of us. we should be at x=2, y=5, z=1.
         */
        Translation3d robot = test (target_051, 2, 0, 0, 180, 0, new Translation3d(2, 5, 1));
        System.out.println(robot);
    }

    @Test
    public void test01() {
        /*
         * We are facing blue wall, camera tipped @ 45°, and we think target is √2 meters in
         * front of us. we should be at x=1, y=5, z=0.
         */
        Translation3d robot = test (target_051, S2, 0, 0, 180, 45, new Translation3d(1, 5, 0));
        System.out.println(robot);
    }

    @Test
    public void test02() {
        /*
         * We are facing away from scoring table, camera tipped up @ 45°, and we think target is √2 meters in
         * front of us. we should be at x=0, y=4, z=0.
         */
        Translation3d robot = test (target_051, S2, 0, 0, 90, 45, new Translation3d(0, 4, 0));
        System.out.println(robot);
    }

    @Test
    public void test03() {
        /*
         * We are facing away from scoring table, camera tipped down @ 45°, and we think target is √2 meters in
         * front of us. we should be at x=0, y=4, z=2.
         */
        Translation3d robot = test (target_051, S2, 0, 0, 90, -45, new Translation3d(0, 4, 2));
        System.out.println(robot);
    }

    @Test
    public void test04() {
        /*
         * We are facing at NW corner of field, camera level, and we think target is √2 meters in
         * front of us. we should be at x=1, y=4, z=1.
         */
        Translation3d robot = test (target_051, S2, 0, 0, 135, 0, new Translation3d(1, 4, 1));
        System.out.println(robot);
    }

    @Test
    public void test05() {
        /*
         * We are facing NW corner of field, up @ 45°, and we think target is √2 meters in
         * front of us. we should be at x=√2/2, y=5-√2/2, z=0.
         */
        Translation3d robot = test (target_051, S2, 0, 0, 135, 45, new Translation3d(S2/2, 5-S2/2, 0));
        System.out.println(robot);
    }

    @Test
    public void test06() {
        /*
         * We are facing NW of field, up @ 35.266°, and we think target is √3 meters in
         * front of us. we should be at x=√2/2, y=5-√2/2, z=0.
         */
        Translation3d robot = test (target_051, S3, 0, 0, 135, 35.266, new Translation3d(1, 4, 0));
        System.out.println(robot);
    }

    @Test
    public void test10() {
        /*
         * We are facing dead W, level, and we think target is 2 meters in
         * front of us and 1 meter above us. we should be at x=2, y=5, z=0.
         */
        Translation3d robot = test (target_051, 2, 0, 1, 180, 0, new Translation3d(2, 5, 0));
        System.out.println(robot);
    }

    Translation3d test (Translation3d target, double x, double y, double z, double robotHeadingInDegrees, double cameraTiltUpInDegrees, Translation3d expected) {
        Translation3d targetFromCamera = new Translation3d(x, y, z);
        Translation3d rv = FieldCalculations.locateCameraViaTarget(target, targetFromCamera, Units.degreesToRadians(robotHeadingInDegrees), Units.degreesToRadians(cameraTiltUpInDegrees));
        if (expected != null) {
            try {
                Assertions.assertEquals(rv.getX(), expected.getX(), 0.01, "X bad");
                Assertions.assertEquals(rv.getY(), expected.getY(), 0.01, "Y bad");
                Assertions.assertEquals(rv.getZ(), expected.getZ(), 0.01, "Z bad");
            } catch (AssertionError e) {
                System.err.println ("Bad result: " + rv + ", expected " + expected);
                throw e;
            }
        }
        return rv;
    }

}
