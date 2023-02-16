package frc.robot;

import org.usfirst.frc3620.misc.PoseOnField;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.VisionSubsystem;

public class TargetPoseOnField extends PoseOnField {

    int redFiducialId, blueFiducialId;
    Translation2d redTargetTranslation, blueTargetTranslation, offsetTranslation;

    TargetPoseOnField (int redFiducialId, int blueFiducialId, double x_inside_offset, double y_offset) {
        this.redFiducialId = redFiducialId;
        this.blueFiducialId = blueFiducialId;
        this.redTargetTranslation = VisionSubsystem.getTranslation3dForTag(redFiducialId).toTranslation2d();
        this.blueTargetTranslation = VisionSubsystem.getTranslation3dForTag(blueFiducialId).toTranslation2d();
        if (x_inside_offset != 0 || y_offset != 0) this.offsetTranslation = new Translation2d(x_inside_offset, y_offset);
    }

    @Override
    public Translation2d getTranslationInMeters(Alliance alliance) {
        Translation2d rv, fixedOffsetTranslation = null;
        switch (alliance) {
            case Blue:
                rv = blueTargetTranslation;
                if (offsetTranslation != null) {
                    fixedOffsetTranslation = new Translation2d(-offsetTranslation.getX(), offsetTranslation.getY());
                }
                break;
            case Red:
                rv = redTargetTranslation;
                fixedOffsetTranslation = offsetTranslation;
                break;
            default:
                throw new IllegalArgumentException("Unsupported enum value");
        }
        if (offsetTranslation != null) rv = rv.plus(fixedOffsetTranslation);
        return rv;
    }

    public int getTargetId() {
        return getTargetId(DriverStation.getAlliance());
    }

    public int getTargetId(Alliance alliance) {
        switch (alliance) {
            case Blue:
                return blueFiducialId;
            case Red:
                return redFiducialId;
            default:
                throw new IllegalArgumentException("Unsupported enum value");
        }
    }

    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder(getClass().getSimpleName());
        sb.append("[red=");
        sb.append(Integer.toString(redFiducialId));
        sb.append(",blue=");
        sb.append(Integer.toString(blueFiducialId));
        if (offsetTranslation != null) {
            sb.append(",x_offset=");
            sb.append(Double.toString(offsetTranslation.getX()));
            sb.append(",y_offset=");
            sb.append(Double.toString(offsetTranslation.getY()))
        }
        sb.append("]");
        return sb.toString();
    }

    static TargetPoseOnField wallTarget = wallTarget(0, 0);
    static TargetPoseOnField midTarget = midTarget(0, 0);
    static TargetPoseOnField humanTarget = humanTarget(0, 0);

    public static TargetPoseOnField wallTarget() {
        return wallTarget;
    }

    public static TargetPoseOnField midTarget() {
        return midTarget;
    }

    public static TargetPoseOnField humanTarget() {
        return humanTarget;
    }

    public static TargetPoseOnField wallTarget(double x, double y) {
        return new TargetPoseOnField(1, 8, x, y);
    }

    public static TargetPoseOnField midTarget(double x, double y) {
        return new TargetPoseOnField(2, 7, x, y);
    }
    
    public static TargetPoseOnField humanTarget(double x, double y) {
        return new TargetPoseOnField(3, 6, x, y);
    }

}
