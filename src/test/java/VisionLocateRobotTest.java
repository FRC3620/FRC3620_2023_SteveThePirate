import java.io.IOException;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import org.usfirst.frc3620.misc.FieldCalculations;

import frc.robot.FieldLayout;
import frc.robot.subsystems.VisionSubsystem;

public class VisionLocateRobotTest {
    /*
     * +Y
     * ^
     * ╟─────────┐
     * ║ NW N NE │
     * Blue Wall ║ W . E │ Red Wall
     * ║ SW S SE │
     * Y=0 ═╬═════════╧═> +X
     * X=0 ▄▄▄
     * Scorer's
     * table
     *
     * origin in SW corner of field (blue alliance wall).
     * Heading 0 points to red alliance wall. Increasing angle CCW.
     * 
     */

    static AprilTagFieldLayout aprilTagFieldLayout;

    @BeforeAll
    public static void init() throws IOException {
        HAL.initialize(500, 0);
        aprilTagFieldLayout = FieldLayout.getAprilTag2023FieldLayout();
    }

    @Test
    public void test01_red() {
        test_red(1);
        test_red(2);
        test_red(3);
    }

    public void test_red(int targetId) {
        Translation3d vectorFromOriginToTag = aprilTagFieldLayout.getTagPose(targetId).get().getTranslation();

        double expected_x = vectorFromOriginToTag.getX() - 1.0;
        double expected_y = vectorFromOriginToTag.getY() - VisionSubsystem.CAMERA_Y_OFFSET;

        Translation2d rv = tester(vectorFromOriginToTag, 1.0, 0, 0, expected_x, expected_y);
        System.err.println ("target " + targetId + ": " + rv);
    }

    @Test
    public void test02_blue() {
        test_blue(6);
        test_blue(7);
        test_blue(8);
    }

    public void test_blue(int targetId) {
        Translation3d vectorFromOriginToTag = aprilTagFieldLayout.getTagPose(targetId).get().getTranslation();

        double expected_x = vectorFromOriginToTag.getX() + 1.0;
        double expected_y = vectorFromOriginToTag.getY() + VisionSubsystem.CAMERA_Y_OFFSET;
        Translation2d rv = tester(vectorFromOriginToTag, 1, 0, 3.14159, expected_x, expected_y);
        System.err.println ("target " + targetId + ": " + rv);
    }

    Translation2d tester(Translation3d vectorFromOriginToTag, double x, double y, double fieldHeadingInRadians, double expected_x, double expected_y) {
        Translation2d vectorFromCameraToTag = new Translation2d(x, y);
        Translation2d rv = VisionSubsystem.calculateCenterOfRobot(vectorFromCameraToTag, vectorFromOriginToTag, fieldHeadingInRadians);

        try {
            Assertions.assertEquals(rv.getX(), expected_x, 0.01, "X bad");
            Assertions.assertEquals(rv.getY(), expected_y, 0.01, "Y bad");
        } catch (AssertionError e) {
            Translation2d expected = new Translation2d(expected_x, expected_y);
            System.err.println("Bad result: " + rv + ", expected " + expected);
            throw e;
        }

        return rv;
    }

}
