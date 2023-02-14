// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class DriveToAprilTagCommand extends CommandBase {
  boolean end = false;
  DriveSubsystem driveSubsystem;
  VisionSubsystem visionSubsystem;

  /** Creates a new DriveToAprilTagCommand. */
  public DriveToAprilTagCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
    this.driveSubsystem = RobotContainer.driveSubsystem;
    this.visionSubsystem = RobotContainer.visionSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveSubsystem.setWheelsToStrafe(0);
    end = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    visionSubsystem.get
    Double tagYaw = visionSubsystem.getTargetYaw();
    Double tagPitch = visionSubsystem.getTargetPitch();
    double targetYaw = 5.8483;
    double targetPitch = -11.304;
    double speed;

    if(tagYaw < targetYaw){
      speed = -0.2;
    }
    else{
      speed = 0.2;
    }

    if (tagYaw == null || tagYaw > 5.01 && tagYaw < 6.6) {
      driveSubsystem.stopDrive();
    } else {
      driveSubsystem.strafeSideways(speed);
    }

    if(tagPitch < targetPitch){
      speed = -0.2;
    }
    else{
      speed = 0.2;
    }

    if (tagPitch == null || tagPitch > -12 && tagPitch < -10) {
      driveSubsystem.stopDrive();
      if (tagPitch > -12 && tagPitch < -10) {
        end = true;
      }
      // do nothing
    } else {
      driveSubsystem.autoDrive(RobotContainer.navigationSubsystem.getCorrectedHeading(), speed, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.stopDrive();
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return end;
  }
}
