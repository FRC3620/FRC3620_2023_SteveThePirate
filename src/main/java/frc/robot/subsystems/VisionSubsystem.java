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
import org.usfirst.frc3620.logger.AsyncDataLogger;
import org.usfirst.frc3620.logger.AsyncDataLoggerDatum;
import org.usfirst.frc3620.logger.LoggingMaster;
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
  PhotonCamera lifecam;
  PhotonPoseEstimator lifecamPoseEstimator;
  static AprilTagFieldLayout fieldLayout;

  public VisionSubsystem() {
    super();
    try {
      fieldLayout = FieldLayout.getAprilTag2023FieldLayout();
    } catch (IOException ex) {
      System.out.println("unable to load file");
    }

    lifecam = new PhotonCamera("Lifecam");
    Transform3d camera1_mounting = new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0,0,0));
    lifecamPoseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.LOWEST_AMBIGUITY, lifecam, camera1_mounting);
  }

  public double atag1TransformX;
  public double atag1TransformY;
  public double atag1TransformZ;
  public boolean getTargetTransform = true;

  public static Double targetOneX = null;

  List<Integer> tags = new ArrayList<>();

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

  @Override
  public void periodic() {
    var result = lifecam.getLatestResult();
    var ts = result.getTimestampSeconds();
    if (lastAprilTagTimestamp != ts) {
      lastAprilTagTimestamp = ts;
      VisionData visionData = null;
      if (visionDataLogger != null) {
        visionData = new VisionData(Timer.getFPGATimestamp() - visionDataLoggerT0, result);
      }

      tags.clear();
      if (result.hasTargets()) {
        Rotation2d whichWayAreWeFacing = RobotContainer.navigationSubsystem.getOdometryHeading(DriverStation.getAlliance());

        PhotonTrackedTarget bestTarget = result.getBestTarget();
        int bestTargetId = bestTarget.getFiducialId();
        List<PhotonTrackedTarget> targetsToProcess;
        if (visionDataLogger != null) {
          targetsToProcess = result.targets;
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

        for (var target : result.targets) {
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

  public Transform3d tag1Transform;

  public Transform3d getTag1Transform() {
    return tag1Transform;
  }

  public void clearTag1Transform() {
    tag1Transform = null;
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