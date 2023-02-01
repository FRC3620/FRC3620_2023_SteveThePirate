// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.usfirst.frc3620.misc.FieldCalculations;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.OdometrySubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class ResetOdometryCommand extends CommandBase {
  /** Creates a new ResetOdometryCommand. */
  public ResetOdometryCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    VisionSubsystem.AllAprilTagsInPicture allAprilTagsInPicture = VisionSubsystem.allAprilTagsInPicture;
    Integer idOfClosetTag = allAprilTagsInPicture.getIdOfClosestTag();
    Transform3d vectorFromCameraToTag = allAprilTagsInPicture.getTransform3d(idOfClosetTag);
    Translation2d vectorToTarget = new Translation2d(vectorFromCameraToTag.getZ(), -vectorFromCameraToTag.getX());

    Translation3d vectorFromOriginToTag = VisionSubsystem.getTranslation3dForTag(idOfClosetTag);
    Rotation2d whichWayAreWeFacing = RobotContainer.navigationSubsystem.getOdometryHeading(DriverStation.getAlliance());

    if (vectorFromOriginToTag != null){
      Translation2d whereIsTheCamera = FieldCalculations.locateCameraViaTarget (vectorFromOriginToTag.toTranslation2d(), vectorToTarget, whichWayAreWeFacing.getRadians());
      Translation2d whereIsTheCameraInches = whereIsTheCamera.times(Units.metersToInches(1));
      OdometrySubsystem.resetPosition(DriverStation.getAlliance(), whereIsTheCamera);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
