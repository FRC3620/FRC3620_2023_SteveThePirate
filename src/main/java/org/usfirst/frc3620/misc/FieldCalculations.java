package org.usfirst.frc3620.misc;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

/** Add your docs here. */
public class FieldCalculations {

    /**
     * https://www.thecivilengineer.org/education/professional-examinations-preparation/calculation-examples/calculation-example-find-the-coordinates-of-the-intersection-of-two-lines
     * @param ta
     * @param ta_heading_radians
     * @param tb
     * @param tb_heading_radians
     * @return
     */
    public static Translation2d locateViaTriangulation (Translation2d ta, double ta_heading_radians, Translation2d tb, double tb_heading_radians) {
        double tan_a = Math.tan(ta_heading_radians);
        double tan_b = Math.tan(tb_heading_radians);
        return locateViaTriangulationGivenTan(ta, tan_a, tb, tan_b);
    }
     
    public static Translation2d locateViaTriangulationGivenTan (Translation2d ta, double tan_a, Translation2d tb, double tan_b) {
        double x_a = ta.getX();
        double y_a = ta.getY();
        double x_b = tb.getX();
        double y_b = tb.getY();

        System.out.println ("tans " + tan_a + " " + tan_b);

        if (Double.isInfinite(tan_a) && Double.isInfinite(tan_b)) {
            return null;
        }
        if (tan_a == tan_b) {
            return null;
        }

        double x = 0.0;
        double y = 0.0;
        if (Double.isFinite(tan_b)) {
            // tan_b is usable
            if (Double.isInfinite(tan_a)) {
                // line to t_a is vertical
                x = x_a;
            } else {
                x = (y_a - y_b - tan_a*x_a + tan_b*x_b) / (tan_b - tan_a);
            }
            y = y_b + tan_b * (x - x_b);
        } else {
            // line to t_b is vertical
            x = x_b;
            y = y_a + tan_a * (x - x_a);
        }
        return new Translation2d(x, y);
    }

    /**
     * 
     * @param target
     * @param x
     * @param y
     * @param robotHeadingInRadians in radians
     * @return
     */
    public static Translation2d locateCameraViaTarget (Translation2d target, double x, double y, double robotHeadingInRadians) {
        Translation2d robotToTargetTranslationRelative = new Translation2d(x, y);
        return locateCameraViaTarget(target, robotToTargetTranslationRelative, robotHeadingInRadians);
    }

    public static Translation2d locateCameraViaTarget (Translation2d target, Translation2d robotToTargetTranslationRelative, double robotHeadingInRadians) {
        Translation2d robotToTargetTranslationAbsolute = robotToTargetTranslationRelative.rotateBy(Rotation2d.fromRadians(robotHeadingInRadians));
        Translation2d targetToRobotTranslationAbsolute = robotToTargetTranslationAbsolute.unaryMinus();
        Translation2d rv = target.plus(targetToRobotTranslationAbsolute);
        return rv;
    }

    public static Translation3d locateCameraViaTarget (Translation3d target, Translation3d targetFromCamera, double robotHeadingInRadians, double cameraTiltUpInRadians) {
        Translation3d robotToTargetTranslationRotated = targetFromCamera.rotateBy(new Rotation3d(0, -cameraTiltUpInRadians, robotHeadingInRadians));
        Translation3d targetToRobotTranslation = robotToTargetTranslationRotated.unaryMinus();
        Translation3d rv = target.plus(targetToRobotTranslation);
        return rv;
    }

}
