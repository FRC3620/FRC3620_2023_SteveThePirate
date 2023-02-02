package frc.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.usfirst.frc3620.misc.FieldCalculations;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class VisionSubsystem extends SubsystemBase {
  PhotonCamera camera1;
  PhotonPoseEstimator camera1PoseEstimator;

  public VisionSubsystem() {
    super();
    try {
      fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
    } catch (IOException ex) {
      System.out.println("unable to load file");
    }

    camera1 = new PhotonCamera("camera1");
    Transform3d camera1_mounting = new Transform3d(new Translation3d(0, 0, 0), new Rotation3d(0,0,0));
    camera1PoseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.LOWEST_AMBIGUITY, camera1, camera1_mounting);
  }

  public double atag1TransformX;
  public double atag1TransformY;
  public double atag1TransformZ;
  public boolean getTargetTransform = true;

  public static Double targetOneX = null;

  static AprilTagFieldLayout fieldLayout;

  List<Integer> tags = new ArrayList<>();

  @Override
  public void periodic() {
    var result = camera1.getLatestResult();
    tags.clear();
    if (result.hasTargets()) {
      PhotonTrackedTarget bestTarget = result.getBestTarget();
      Transform3d vectorFromCameraToTag = bestTarget.getBestCameraToTarget();
      int idOfBestTag = bestTarget.getFiducialId();
      SmartDashboard.putNumber("whereami.closest tag id", bestTarget.getFiducialId());
      SmartDashboard.putNumber("whereami.closest tag distance x", vectorFromCameraToTag.getX());
      SmartDashboard.putNumber("whereami.closest tag distance y", vectorFromCameraToTag.getY());
      SmartDashboard.putNumber("whereami.closest tag distance z", vectorFromCameraToTag.getZ());

      Translation2d vectorToTarget = new Translation2d(vectorFromCameraToTag.getZ(), -vectorFromCameraToTag.getX());

      SmartDashboard.putNumber("Translated X", vectorToTarget.getX());
      SmartDashboard.putNumber("Translated Y", vectorToTarget.getY());
        
      Translation3d vectorFromOriginToTag = VisionSubsystem.getTranslation3dForTag(idOfBestTag);
      Rotation2d whichWayAreWeFacing = RobotContainer.navigationSubsystem.getOdometryHeading(DriverStation.getAlliance());
      SmartDashboard.putNumber("whereami.facing", whichWayAreWeFacing.getDegrees());

      if (vectorFromOriginToTag != null){
        Translation2d whereIsTheCamera = FieldCalculations.locateCameraViaTarget (vectorFromOriginToTag.toTranslation2d(), vectorToTarget, whichWayAreWeFacing.getRadians());
        Translation2d whereIsTheCameraInches = whereIsTheCamera.times(Units.metersToInches(1));
        SmartDashboard.putNumber("camera X", whereIsTheCameraInches.getX());
        SmartDashboard.putNumber("camera Y", whereIsTheCameraInches.getY());
      }

      for (var target : result.targets) {
        tags.add(target.getFiducialId());
      }
    }
    SmartDashboard.putString("tag", tags.toString());
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
      SmartDashboard.putNumber("whereami.TagId", tag);
      SmartDashboard.putNumber("whereami.TagPosex", transform3d.getX());
      SmartDashboard.putNumber("whereami.TagPosey", transform3d.getY());
      SmartDashboard.putNumber("whereami.TagPosez", transform3d.getZ());
      return transform3d;
    }
    return null;
  }
}