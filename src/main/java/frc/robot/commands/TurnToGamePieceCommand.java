// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.INavigationSubsystem;
import frc.robot.subsystems.NavXNavigationSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class TurnToGamePieceCommand extends CommandBase {
  /** Creates a new TurnToGamePieceCommand. */
  private DriveSubsystem driveSubsystem;
  private VisionSubsystem visionSubsystem;
  double currentYaw;
  double currentHeading;
  double targetHeading;
  double tolerance = 5;
  double lastTimestamp;
  public TurnToGamePieceCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSubsystem = driveSubsystem;
    this.visionSubsystem = visionSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveSubsystem.setForcedManualModeTrue();
    SmartDashboard.putBoolean("TurntoGamePiece.running", true);
    lastTimestamp = -1;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    PhotonPipelineResult result = visionSubsystem.getLastFrontCameraGamePieceResult(lastTimestamp);
    if (result != null) {
      lastTimestamp = result.getTimestampSeconds();
      PhotonTrackedTarget target = result.getBestTarget();
      double spinPower = 0.3;
      currentYaw = target.getYaw();
      currentHeading = RobotContainer.navigationSubsystem.getCorrectedHeading();
      targetHeading = currentHeading + currentYaw;

      SmartDashboard.putNumber("gamepiece.yaw", currentYaw);
      SmartDashboard.putNumber("gamepiece.targetHeading", targetHeading);
      SmartDashboard.putNumber("gamepiece.currentHeading", currentHeading);

      if(currentYaw < 0){
        spinPower = -spinPower;
      }

      driveSubsystem.setTargetHeading(targetHeading);

      driveSubsystem.autoDrive(currentYaw, 0, spinPower);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(currentHeading > targetHeading - tolerance && currentHeading < targetHeading + tolerance){
      return true;
    }
    else{
    return false;
    }
  }
}
