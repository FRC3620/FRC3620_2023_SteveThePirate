package frc.robot;

import org.usfirst.frc3620.misc.PoseOnField;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.VisionSubsystem;

public class TargetPoseOnField extends PoseOnField {

    int redFiducialId, blueFiducialId;
    Translation2d redTargetPose, blueTargetPose;

    TargetPoseOnField (int redFiducialId, int blueFiducialId) {
        this.redFiducialId = redFiducialId;
        this.blueFiducialId = blueFiducialId;
        this.redTargetPose = VisionSubsystem.getTranslation3dForTag(redFiducialId).toTranslation2d();
        this.blueTargetPose = VisionSubsystem.getTranslation3dForTag(blueFiducialId).toTranslation2d();
    }

    @Override
    public Translation2d getTranslationInMeters(Alliance alliance) {
        switch (alliance) {
            case Blue:
                return blueTargetPose;
            case Red:
                return redTargetPose;
            default:
                throw new IllegalArgumentException("Unsupported enum value");
        }
    }

    /**
     * return an absolute field position in front of this target.
     * x is distance in metersfrom the target, positive heading toward
     * @param alliance whic alliance we are
     * @param x x distance in meters from target, positive heading toward
     * center of the field
     * @param y y distance in meters from target, positive heading away 
     * the wall and scorer's table
     * @return 
     */
    public Translation2d inFront(Alliance alliance, double x, double y) {
        Translation2d t2d_target;
        switch (alliance) {
            case Blue:
                t2d_target = blueTargetPose;
                break;
            case Red:
                t2d_target = redTargetPose;
                x = -x;
                break;
            default:
                throw new IllegalArgumentException("Unsupported enum value");
        }
        Translation2d rv = t2d_target.plus(new Translation2d(x, y));
        return rv;
    }

    public Translation2d inFront(Alliance alliance, Translation2d t2d) {
        return inFront(alliance, t2d.getX(), t2d.getY());
    }

    public Translation2d inFront(Translation2d t2d) {
        return inFront(DriverStation.getAlliance(), t2d);
    }

    public Translation2d inFront(double x, double y) {
        return inFront(DriverStation.getAlliance(), x, y);
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
        sb.append("]");
        return sb.toString();
    }

    static public TargetPoseOnField WallTarget = new TargetPoseOnField(1, 8);
    static public TargetPoseOnField MidTarget = new TargetPoseOnField(2, 7);
    static public TargetPoseOnField HumanTarget = new TargetPoseOnField(3, 6);
}
