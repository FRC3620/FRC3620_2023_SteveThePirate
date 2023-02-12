package frc.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.usfirst.frc3620.misc.FieldCalculations;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldLayout;
import frc.robot.RobotContainer;

public class VisionSubsystem extends SubsystemBase {
  public enum FrontCameraMode {
    APRILTAGS(0), CONES(1), CUBES(2);

    int pipelineIndex;

    FrontCameraMode(int pipeline) {
      this.pipelineIndex = pipeline;
    }

    public int getPipelineIndex() {
      return pipelineIndex;
    }
  }

  PhotonCamera frontCamera;
  PhotonPoseEstimator frontCameraPoseEstimator;
  FrontCameraMode frontCameraMode;

  static AprilTagFieldLayout fieldLayout;

  public VisionSubsystem() {
    super();
    try {
      fieldLayout = FieldLayout.getAprilTag2023FieldLayout();
    } catch (IOException ex) {
      System.out.println("unable to load file");
    }

    frontCamera = new PhotonCamera("Lifecam");
    setFrontCameraMode(FrontCameraMode.APRILTAGS);
    Transform3d frontCameraMounting = new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0,0,0));
    frontCameraPoseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.LOWEST_AMBIGUITY, frontCamera, frontCameraMounting);
  }

  public FrontCameraMode setFrontCameraMode(FrontCameraMode mode) {
    FrontCameraMode oldMode = this.frontCameraMode;
    frontCamera.setPipelineIndex(mode.getPipelineIndex());
    return oldMode;
  }

  public FrontCameraMode getFrontCameraMode() {
    return this.frontCameraMode;
  }

  public PhotonTrackedTarget getTargetById(PhotonPipelineResult result, int requestedId) {
    if (result.hasTargets()) {
      for (var target: result.getTargets()) {
        if (target.getFiducialId() == requestedId) {
          return target;
        }
      }
    }
    return null;
  }

  @Override
  public void periodic() {
    if (frontCameraMode == FrontCameraMode.APRILTAGS) {
      aprilTagsPeriodic();
    } else {
      gamePiecePeriodic();
    }
  }

  PhotonPipelineResult lastFrontCameraAprilTagsResult;

  public PhotonPipelineResult getLastFrontCameraAprilTagsResult() {
    return lastFrontCameraAprilTagsResult;
  }

  public PhotonPipelineResult getLastFrontCameraAprilTagsResult(double timestampSeconds) {
    if (lastFrontCameraAprilTagsResult != null && lastFrontCameraAprilTagsResult.getTimestampSeconds() != timestampSeconds) {
      return lastFrontCameraAprilTagsResult;
    }
    return null;
  }

  List<Integer> tags = new ArrayList<>();
  void aprilTagsPeriodic() {
    lastFrontCameraAprilTagsResult = frontCamera.getLatestResult();
    tags.clear();
    if (lastFrontCameraAprilTagsResult.hasTargets()) {
      PhotonTrackedTarget bestTarget = lastFrontCameraAprilTagsResult.getBestTarget();
      Transform3d vectorFromCameraToTag = bestTarget.getBestCameraToTarget();
      int idOfBestTag = bestTarget.getFiducialId();
      SmartDashboard.putNumber("whereami.closest tag id", bestTarget.getFiducialId());
      SmartDashboard.putNumber("whereami.closest tag distance x", vectorFromCameraToTag.getX());
      SmartDashboard.putNumber("whereami.closest tag distance y", vectorFromCameraToTag.getY());
      SmartDashboard.putNumber("whereami.closest tag distance z", vectorFromCameraToTag.getZ());

      //Translation2d vectorToTarget = new Translation2d(vectorFromCameraToTag.getZ(), -vectorFromCameraToTag.getX());
      Translation2d vectorToTarget = vectorFromCameraToTag.getTranslation().toTranslation2d();

      SmartDashboard.putNumber("Translated X", vectorToTarget.getX());
      SmartDashboard.putNumber("Translated Y", vectorToTarget.getY());
        
      Translation3d vectorFromOriginToTag = VisionSubsystem.getTranslation3dForTag(idOfBestTag);
      if (vectorFromOriginToTag != null) {
        SmartDashboard.putNumber("whereami.TagPosex", vectorFromOriginToTag.getX());
        SmartDashboard.putNumber("whereami.TagPosey", vectorFromOriginToTag.getY());
        SmartDashboard.putNumber("whereami.TagPosez", vectorFromOriginToTag.getZ());

        Rotation2d whichWayAreWeFacing = RobotContainer.navigationSubsystem.getOdometryHeading(DriverStation.getAlliance());
        SmartDashboard.putNumber("whereami.facing", whichWayAreWeFacing.getDegrees());

        Translation2d whereIsTheCamera = FieldCalculations.locateCameraViaTarget (vectorFromOriginToTag.toTranslation2d(), vectorToTarget, whichWayAreWeFacing.getRadians());
        SmartDashboard.putNumber("camera X", whereIsTheCamera.getX());
        SmartDashboard.putNumber("camera Y", whereIsTheCamera.getY());
        RobotContainer.odometrySubsystem.resetPosition(DriverStation.getAlliance(), whereIsTheCamera);
      }

      for (var target : lastFrontCameraAprilTagsResult.targets) {
        tags.add(target.getFiducialId());
      }
    }
    SmartDashboard.putString("tags", tags.toString());
  }

  PhotonPipelineResult lastFrontCameraGamePieceResult;

  public PhotonPipelineResult getLastFrontCameraGamePieceResult() {
    return lastFrontCameraGamePieceResult;
  }

  public PhotonPipelineResult getLastFrontCameraGamePieceResult(double timestampSeconds) {
    if (lastFrontCameraGamePieceResult != null && lastFrontCameraGamePieceResult.getTimestampSeconds() != timestampSeconds) {
      return lastFrontCameraGamePieceResult;
    }
    return null;
  }

  void gamePiecePeriodic() {
    lastFrontCameraGamePieceResult = frontCamera.getLatestResult();
  }

  public static Translation3d getTranslation3dForTag(int tag) {
    var tagPoseOptional = fieldLayout.getTagPose(tag);
    if (tagPoseOptional.isPresent()) {
      Translation3d transform3d = tagPoseOptional.get().getTranslation();
      return transform3d;
    }
    return null;
  }
}