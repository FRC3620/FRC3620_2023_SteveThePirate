// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.FrontCameraMode;

public class DriveToGamePieceCommand extends CommandBase {
  DriveSubsystem driveSubsystem;
  VisionSubsystem visionSubsystem;
  double tolerance = 5;
  double lastTimestamp;
  FrontCameraMode currentCameraMode;
  FrontCameraMode pipeline;

  double targetHeading;

  enum MyState {
    SEARCHING, DRIVING, STOPPED
  }

  MyState myState;

  /** Creates a new DriveToGamePieceCommand. */
  public DriveToGamePieceCommand(FrontCameraMode pipeline, DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.visionSubsystem = visionSubsystem;
    this.pipeline = pipeline;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem, visionSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    myState = MyState.SEARCHING;
    // I don't think we want this
    // driveSubsystem.setForcedManualModeTrue();
    lastTimestamp = -1;
    currentCameraMode = visionSubsystem.setFrontCameraMode(pipeline);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    PhotonPipelineResult result = visionSubsystem.getLastFrontCameraGamePieceResult(lastTimestamp);
    SmartDashboard.putBoolean("gamepiece.have_vision_result", result != null);
    PhotonTrackedTarget target = null;
    if (result == null) {
      SmartDashboard.putBoolean("gamepiece.have_vision_target", false);
    } else {
      // remember that we got a result!
      lastTimestamp = result.getTimestampSeconds();

      if (result.hasTargets()) {
        target = result.getBestTarget();
      
        int id = target.getFiducialId();
        SmartDashboard.putNumber("gamepiece.target_id", id);
        if (id != -1) {
          // target is probably from AprilTags
          target = null;
        }
      }
    
      if (target == null) {
        driveSubsystem.stopDrive();
      } else {
        double currentHeading = RobotContainer.navigationSubsystem.getCorrectedHeading();
        double currentYaw = target.getYaw();
        double currentPitch = target.getPitch();
        SmartDashboard.putNumber("gamepiece.yaw", currentYaw);
        SmartDashboard.putNumber("gamepiece.pitch", currentPitch);

        if (myState == MyState.SEARCHING) {
          double spinPower = currentYaw * 0.03;
          spinPower = MathUtil.clamp(spinPower, -.3, .3);
          double targetYaw = 11.8;

          targetHeading = currentHeading + currentYaw;
          SmartDashboard.putNumber("gamepiece.targetHeading", targetHeading);

          driveSubsystem.autoDrive(currentYaw, 0, spinPower);
          if (currentHeading > targetHeading - tolerance && currentHeading < targetHeading + tolerance) {
            driveSubsystem.setTargetHeading(targetHeading);
            myState = MyState.DRIVING;
          }
        } else if (myState == MyState.DRIVING) {
          double spinPower = driveSubsystem.getSpinPower();
          driveSubsystem.autoDrive(targetHeading - currentHeading, 0.2, spinPower);
          if (currentPitch < 1.6 && currentPitch > 0) { // MAY CHANGE NUM
            myState = MyState.STOPPED;
          }
        }
      }
    }
    SmartDashboard.putString("gamepiece.state", myState.toString());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    visionSubsystem.setFrontCameraMode(currentCameraMode);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (myState == MyState.STOPPED) {
      return true;
    } else {
      return false;
    }
  }
}
