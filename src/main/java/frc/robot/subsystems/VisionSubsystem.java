package frc.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Date;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.slf4j.Logger;
import org.usfirst.frc3620.logger.AsyncDataLogger;
import org.usfirst.frc3620.logger.AsyncDataLoggerDatum;
import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.logger.LoggingMaster;
import org.usfirst.frc3620.logger.EventLogging.Level;
import org.usfirst.frc3620.misc.FieldCalculations;

import com.fasterxml.jackson.annotation.JsonIgnore;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldLayout;
import frc.robot.RobotContainer;

public class VisionSubsystem extends SubsystemBase {
  public enum FrontCameraMode {
    APRILTAGS(2), CONES(1), CUBES(0);

    int pipelineIndex;

    FrontCameraMode(int pipeline) {
      this.pipelineIndex = pipeline;
    }

    public int getPipelineIndex() {
      return pipelineIndex;
    }
  }

  Logger logger = EventLogging.getLogger(getClass(), Level.INFO);
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

    frontPipelineCheckTimer = new Timer();
    frontPipelineCheckTimer.start();
  }

  public FrontCameraMode setFrontCameraMode(FrontCameraMode mode) {
    FrontCameraMode oldMode = this.frontCameraMode;
    frontCamera.setPipelineIndex(mode.getPipelineIndex());
    this.frontCameraMode = mode;
    logger.info ("Changed front camera mode from {} to {}", oldMode, mode);
    return oldMode;
  }

  public FrontCameraMode getFrontCameraMode() {
    return this.frontCameraMode;
  }

  public static PhotonTrackedTarget getTargetById(PhotonPipelineResult result, int requestedId) {
    if (result.hasTargets()) {
      for (var target: result.getTargets()) {
        if (target.getFiducialId() == requestedId) {
          return target;
        }
      }
    }
    return null;
  }

  AsyncDataLogger<VisionData> visionDataLogger;
  double visionDataLoggerT0;

  static ObjectMapper objectMapper = new ObjectMapper();

  public void startVisionDataLogger() {
    String filename = "visionandodometry_" + LoggingMaster.convertTimestampToString(new Date()) + ".json";
    visionDataLogger = new AsyncDataLogger<>(filename, 1000);
    visionDataLoggerT0 = Timer.getFPGATimestamp();
  }

  public void doneWithVisionDataLogger() {
    visionDataLogger.close();
    visionDataLogger = null;
  }

  double lastAprilTagTimestamp = -1.0;

  Timer frontPipelineCheckTimer;

  @Override
  public void periodic() {
    if (frontPipelineCheckTimer.advanceIfElapsed(1.0)) {
      int frontCameraPipelineIndex = frontCamera.getPipelineIndex();
      int shouldBe = frontCameraMode.getPipelineIndex();
      if (frontCameraPipelineIndex != shouldBe) {
        logger.warn ("Had to force front camera pipeline from {} to {}", frontCameraPipelineIndex, shouldBe);
        frontCamera.setPipelineIndex(shouldBe);
      }
    }
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
    var ts = lastFrontCameraAprilTagsResult.getTimestampSeconds();
    if (lastAprilTagTimestamp != ts) {
      lastAprilTagTimestamp = ts;
      VisionData visionData = null;
      if (visionDataLogger != null) {
        visionData = new VisionData(Timer.getFPGATimestamp() - visionDataLoggerT0, lastFrontCameraAprilTagsResult);
      }

      tags.clear();
      if (lastFrontCameraAprilTagsResult.hasTargets()) {
        Rotation2d whichWayAreWeFacing = RobotContainer.navigationSubsystem.getOdometryHeading(DriverStation.getAlliance());

        PhotonTrackedTarget bestTarget = lastFrontCameraAprilTagsResult.getBestTarget();
        int bestTargetId = bestTarget.getFiducialId();
        List<PhotonTrackedTarget> targetsToProcess;
        if (visionDataLogger != null) {
          targetsToProcess = lastFrontCameraAprilTagsResult.targets;
        } else {
          targetsToProcess = Collections.singletonList(bestTarget);
        }

        for (var target : targetsToProcess) {
          int targetId = target.getFiducialId();
          Transform3d vectorFromCameraToTag = target.getBestCameraToTarget();
          //Translation2d vectorToTarget = new Translation2d(vectorFromCameraToTag.getZ(), -vectorFromCameraToTag.getX());
          Translation2d vectorToTarget = vectorFromCameraToTag.getTranslation().toTranslation2d();
          Translation3d vectorFromOriginToTag = VisionSubsystem.getTranslation3dForTag(targetId);

          if (targetId == bestTargetId) {
            SmartDashboard.putNumber("whereami.closest tag id", targetId);
            SmartDashboard.putNumber("whereami.closest tag distance x", vectorFromCameraToTag.getX());
            SmartDashboard.putNumber("whereami.closest tag distance y", vectorFromCameraToTag.getY());
            SmartDashboard.putNumber("whereami.closest tag distance z", vectorFromCameraToTag.getZ());
      
            SmartDashboard.putNumber("Translated X", vectorToTarget.getX());
            SmartDashboard.putNumber("Translated Y", vectorToTarget.getY());
          }

          if (vectorFromOriginToTag != null) {
            Translation2d whereIsTheCamera = FieldCalculations.locateCameraViaTarget (vectorFromOriginToTag.toTranslation2d(), vectorToTarget, whichWayAreWeFacing.getRadians());
            if (targetId == bestTargetId) {
              RobotContainer.odometrySubsystem.resetPosition(DriverStation.getAlliance(), whereIsTheCamera);

              SmartDashboard.putNumber("whereami.TagPosex", vectorFromOriginToTag.getX());
              SmartDashboard.putNumber("whereami.TagPosey", vectorFromOriginToTag.getY());
              SmartDashboard.putNumber("whereami.TagPosez", vectorFromOriginToTag.getZ());
      
              SmartDashboard.putNumber("whereami.facing", whichWayAreWeFacing.getDegrees());
      
              SmartDashboard.putNumber("camera X", whereIsTheCamera.getX());
              SmartDashboard.putNumber("camera Y", whereIsTheCamera.getY());
            }
            if (visionData != null) {
              visionData.addCameraPosition(targetId, whereIsTheCamera);
            }
          }
        }

      for (var target : lastFrontCameraAprilTagsResult.targets) {
          tags.add(target.getFiducialId());
        }
      }
      SmartDashboard.putString("tags", tags.toString());

      if (visionDataLogger != null) {
        visionData.setOdometryAfter(RobotContainer.odometrySubsystem.getPoseMeters().getTranslation());
        visionDataLogger.send(visionData);
      }
    }      
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

  public static class VisionData implements AsyncDataLoggerDatum {
    final public double time;
    final public Translation2d odometry, blind_odometry;
    public Translation2d odometry_after;
    final public double robotHeading;
    final public Map<Integer, Translation2d> cameraPositions = new HashMap<>();
    final public PhotonPipelineResult photonPipelineResult;

    VisionData(double t, PhotonPipelineResult photonPipelineResult) {
      this.time = t;
      this.odometry = RobotContainer.odometrySubsystem.getPoseMeters().getTranslation();
      this.blind_odometry = RobotContainer.odometrySubsystem.getBlindPoseMeters().getTranslation();
      this.robotHeading = RobotContainer.navigationSubsystem.getCorrectedHeading();
      this.photonPipelineResult = photonPipelineResult;
    }

    void addCameraPosition(int id, Translation2d t2d) {
      cameraPositions.put(id, t2d);
    }

    void setOdometryAfter(Translation2d t2d) {
      this.odometry_after = t2d;
    }

    @Override
    @JsonIgnore
    public byte[] getAsyncDataLoggerBytes() {
      String s = "";
      try {
        s = objectMapper.writeValueAsString(this);
      } catch (JsonProcessingException e) {
        s = e.toString();
      }
      s += "\n";
      return s.getBytes();
    }

  }
}