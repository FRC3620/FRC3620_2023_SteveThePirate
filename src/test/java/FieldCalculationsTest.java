import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import org.usfirst.frc3620.misc.FieldCalculations;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/** Add your docs here. */
public class FieldCalculationsTest {
    Translation2d t01 = new Translation2d(0, 1);
    Translation2d t10 = new Translation2d(1, 0);

    @Test
    public void test00() {
        Translation2d rv1 = FieldCalculations.locateViaTriangulation(t10, Units.degreesToRadians(-90), t01, Units.degreesToRadians(180));
        Translation2d rv2 = FieldCalculations.locateCameraViaTarget(t10, 1.1, 0, Units.degreesToRadians(-90));
        Translation2d rv3 = FieldCalculations.locateCameraViaTarget(t01, 0.9, 0, Units.degreesToRadians(180));
        Translation2d rv = average(rv1, rv2, rv3);
        System.out.println (rv1);
        System.out.println (rv2);
        System.out.println (rv3);
        System.out.println (rv);
    }

    Translation2d average (Translation2d... t) {
        if (t.length == 0) throw new IllegalArgumentException("zero length argument list");
        double x = 0.0;
        double y = 0.0;
        for (Translation2d t1 : t) {
            x += t1.getX();
            y += t1.getY();
        }
        return new Translation2d(x / t.length, y / t.length);
    }
}
