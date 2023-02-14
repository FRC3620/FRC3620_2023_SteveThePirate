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
import frc.robot.subsystems.VisionSubsystem;

public class DriveToAprilTagCommand extends CommandBase {
  boolean end = false;
  boolean pitchTime = false;
  DriveSubsystem driveSubsystem;
  VisionSubsystem visionSubsystem;
  double lastTimestamp;

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
    pitchTime = false;
    lastTimestamp = -1;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    PhotonPipelineResult result = visionSubsystem.getLastFrontCameraAprilTagsResult(lastTimestamp);
    SmartDashboard.putString("apriltag.result", "" + result);
    if (result != null) {
      lastTimestamp = result.getTimestampSeconds();
      SmartDashboard.putBoolean("apriltag.hasTargets", result.hasTargets());
      if (result.hasTargets())  {
        SmartDashboard.putString("apriltag.targets", result.getTargets().toString());
      }
      PhotonTrackedTarget target = VisionSubsystem.getTargetById(result, 8);
      SmartDashboard.putString("apriltag.target", "" + target);
      if (target != null) {
        // do work here
        Double tagYaw = target.getYaw();
        Double tagPitch = target.getPitch();
        double targetYaw = 5.8483;
        double targetPitch = -11.304;
        double speed;

        SmartDashboard.putNumber("apriltag.tagYaw", tagYaw);
        SmartDashboard.putNumber("apriltag.tagPitch", tagPitch);
    
        if(tagYaw < targetYaw){
          speed = -0.2;
        }
        else{
          speed = 0.2;
        }
    
        if (tagYaw == null || tagYaw > 5.01 && tagYaw < 6.6) {
          driveSubsystem.stopDrive();
          pitchTime = true;
        } else {
          driveSubsystem.strafeSideways(speed);
        }

        SmartDashboard.putNumber("apriltag.speed", speed);

        if(pitchTime = true){
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
      }      
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
