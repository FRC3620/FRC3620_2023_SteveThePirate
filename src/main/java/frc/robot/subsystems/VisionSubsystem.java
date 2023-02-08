package frc.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.dataflow.structures.Packet;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.usfirst.frc3620.logger.AsyncDataLogger;
import org.usfirst.frc3620.logger.AsyncDataLoggerDatum;
import org.usfirst.frc3620.misc.FieldCalculations;

import com.fasterxml.jackson.annotation.JsonIdentityInfo;
import com.fasterxml.jackson.annotation.JsonIgnore;
import com.fasterxml.jackson.annotation.ObjectIdGenerators;
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

  AsyncDataLogger<VisionData> visionDataLogger;

  static ObjectMapper objectMapper = new ObjectMapper();

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

    visionDataLogger = new AsyncDataLogger<>("odometry.json", 1000);
  }

  public double atag1TransformX;
  public double atag1TransformY;
  public double atag1TransformZ;
  public boolean getTargetTransform = true;

  public static Double targetOneX = null;

  List<Integer> tags = new ArrayList<>();

  @Override
  public void periodic() {
    var result = lifecam.getLatestResult();
    tags.clear();
    if (result.hasTargets()) {
      PhotonTrackedTarget bestTarget = result.getBestTarget();
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

      for (var target : result.targets) {
        tags.add(target.getFiducialId());
      }
    }
    SmartDashboard.putString("tags", tags.toString());

    if (visionDataLogger != null) {
      VisionData visionData = new VisionData(result);
      visionDataLogger.send(visionData);
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
    public double when;

    public Integer bestTarget;

    public List<PhotonTrackedTarget> targets;

    public Translation2d odometry;
    
    public double robotHeading;

    VisionData(PhotonPipelineResult photonPipelineResult) {
      this.when = Timer.getFPGATimestamp();
      this.targets = photonPipelineResult.getTargets();
      this.bestTarget = photonPipelineResult.getBestTarget().getFiducialId();
      this.odometry = RobotContainer.odometrySubsystem.getPoseMeters().getTranslation();
      this.robotHeading = RobotContainer.navigationSubsystem.getCorrectedHeading();
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