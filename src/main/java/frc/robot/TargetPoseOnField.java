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

    static public TargetPoseOnField WallTarget = new TargetPoseOnField(1, 8);
    static public TargetPoseOnField MidTarget = new TargetPoseOnField(2, 7);
    static public TargetPoseOnField HumanTarget = new TargetPoseOnField(3, 6);
}
