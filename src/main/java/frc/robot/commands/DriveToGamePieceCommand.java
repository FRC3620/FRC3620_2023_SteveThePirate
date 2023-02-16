// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.INavigationSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class DriveToGamePieceCommand extends CommandBase {
  DriveSubsystem driveSubsystem;
  VisionSubsystem visionSubsystem;
  double currentYaw;
  double currentHeading;
  double targetHeading;
  double tolerance = 5;
  double lastTimestamp;

  enum MyState{
    SEARCHING, DRIVING, STOPPED
  }

  MyState myState;
  
  /** Creates a new DriveToGamePieceCommand. */
  public DriveToGamePieceCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSubsystem = driveSubsystem;
    this.visionSubsystem = visionSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    myState = MyState.SEARCHING;
    driveSubsystem.setForcedManualModeTrue();
    lastTimestamp = -1;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    PhotonPipelineResult result = visionSubsystem.getLastFrontCameraGamePieceResult(lastTimestamp);
    if (result != null) {
      lastTimestamp = result.getTimestampSeconds();
      PhotonTrackedTarget target = result.getBestTarget();
    if(myState == MyState.SEARCHING){
      double spinPower = 0.3;
      currentYaw = target.getYaw();
      currentHeading = RobotContainer.navigationSubsystem.getCorrectedHeading();
      targetHeading = currentHeading + currentYaw;

      if(currentYaw < 0){
        spinPower = -spinPower;
      }

      driveSubsystem.setTargetHeading(targetHeading);

      driveSubsystem.autoDrive(currentYaw, 0, spinPower);
      if(currentHeading > targetHeading - tolerance && currentHeading < targetHeading + tolerance){
        myState = MyState.DRIVING;
      }
    }

    if(myState == MyState.DRIVING){
      driveSubsystem.autoDrive(0, 0.2, 0);
      if(target.getPitch() < -10){ //MAY CHANGE NUM
        myState = MyState.STOPPED;
      }
    }
  }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(myState == MyState.STOPPED){
      return true;
    } else {
      return false;
    }
  }
}
