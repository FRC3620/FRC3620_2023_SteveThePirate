// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Observer;
import java.util.Optional;

import org.usfirst.frc3620.misc.FieldCalculations;

import edu.wpi.first.math.estimator.KalmanFilterLatencyCompensator.ObserverSnapshot;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.VisionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class WhereAmI extends CommandBase {
  public WhereAmI() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  @Override
  public void execute() {
    VisionSubsystem.AllAprilTagsInPicture allAprilTagsInPicture = VisionSubsystem.allAprilTagsInPicture;
    if(allAprilTagsInPicture != null)
    {
      Integer idOfClosetTag = allAprilTagsInPicture.getIdOfClosestTag();
      if(idOfClosetTag != null)
      {
        Transform3d vectorFromCameraToTag = allAprilTagsInPicture.getTransform3d(idOfClosetTag);
        SmartDashboard.putNumber("whereami.closest tag id", idOfClosetTag);
        SmartDashboard.putNumber("whereami.closest tag distance x", vectorFromCameraToTag.getX());
        SmartDashboard.putNumber("whereami.closest tag distance y", vectorFromCameraToTag.getY());
        SmartDashboard.putNumber("whereami.closest tag distance z", vectorFromCameraToTag.getZ());

        Translation2d vectorToTarget = new Translation2d(vectorFromCameraToTag.getZ(), -vectorFromCameraToTag.getX());

        SmartDashboard.putNumber("Translated X", vectorToTarget.getX());
        SmartDashboard.putNumber("Translated Y", vectorToTarget.getY());

        Translation3d vectorFromOriginToTag = VisionSubsystem.getTranslation3dForTag(idOfClosetTag);

        Rotation2d whichWayAreWeFacing = RobotContainer.navigationSubsystem.getOdometryHeading(DriverStation.getAlliance());
        SmartDashboard.putNumber("whereami.facing", whichWayAreWeFacing.getDegrees());
        Translation2d whereIsTheCamera = FieldCalculations.locateCameraViaTarget (vectorFromOriginToTag.toTranslation2d(), vectorToTarget, whichWayAreWeFacing.getRadians());
        SmartDashboard.putNumber("camera X", whereIsTheCamera.getX());
        SmartDashboard.putNumber("camera Y", whereIsTheCamera.getY());



      

      }

    }
  }

  @Override
  public boolean runsWhenDisabled(){
    return true;
  }
}
