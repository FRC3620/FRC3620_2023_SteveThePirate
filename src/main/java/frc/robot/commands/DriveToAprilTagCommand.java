// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.OdometrySubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class DriveToAprilTagCommand extends CommandBase {
  boolean end = false;
  boolean pitchTime = false;
  DriveSubsystem driveSubsystem;
  VisionSubsystem visionSubsystem;
  OdometrySubsystem odometrySubsystem;
  double lastTimestamp;
  int tagID;

  enum MyState {
    SQUARING, STRAFING, FORWARD, LAST
  }

  MyState myState;

  public enum Position{
    MIDDLE, WALL, HUMAN
  }

  Position position;

  /** Creates a new DriveToAprilTagCommand. */
  public DriveToAprilTagCommand(int tagID, Position position, DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, OdometrySubsystem odometrySubsystem) {
    this.tagID = tagID;
    this.position = position;
    this.driveSubsystem = RobotContainer.driveSubsystem;
    this.visionSubsystem = RobotContainer.visionSubsystem;
    this.odometrySubsystem = odometrySubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem, visionSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    myState = MyState.SQUARING;
    driveSubsystem.setWheelsToStrafe(0);
    end = false;
    lastTimestamp = -1;
    double currentHeading = RobotContainer.navigationSubsystem.getCorrectedHeading();
    SmartDashboard.putNumber("apriltag.headingStart", currentHeading);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentHeading = RobotContainer.navigationSubsystem.getCorrectedHeading();

    if (myState == MyState.SQUARING) {
      driveSubsystem.setTargetHeading(180);
      double spinX = driveSubsystem.getSpinPower();
      driveSubsystem.autoDrive(0, 0, spinX);
      if (Math.abs(currentHeading) > 175 && Math.abs(currentHeading) < 185) {
        driveSubsystem.stopDrive();
        myState = MyState.STRAFING;
        SmartDashboard.putNumber("apriltag.headingSquare", currentHeading);
      }
    }
    PhotonPipelineResult result = visionSubsystem.getLastFrontCameraAprilTagsResult(lastTimestamp);
    SmartDashboard.putString("apriltag.result", "" + result);
    if (result != null) {
      lastTimestamp = result.getTimestampSeconds();
      SmartDashboard.putBoolean("apriltag.hasTargets", result.hasTargets());
      if (result.hasTargets()) {
        SmartDashboard.putString("apriltag.targets", result.getTargets().toString());
      }
      PhotonTrackedTarget target = VisionSubsystem.getTargetById(result, tagID);
      SmartDashboard.putString("apriltag.target", "" + target);

      if (target != null) {
        // do work here
        Double tagYaw = target.getYaw();
        Double tagPitch = target.getPitch();
        double targetYaw = 5.8483;
        double targetPitch = -11.304;
        double speed = 0.0;

        SmartDashboard.putString("apriltag.state", myState.toString());

        if (myState == MyState.STRAFING) {

          if (tagYaw < targetYaw) {
            speed = -0.1;
          } else {
            speed = 0.1;
          }

          if (tagYaw == null || tagYaw > 5.01 && tagYaw < 6.6) {
            driveSubsystem.stopDrive();
            myState = MyState.FORWARD;
            SmartDashboard.putNumber("apriltag.headingStrafe", currentHeading);
            SmartDashboard.putNumber("apriltag.tagYawStrafe", tagYaw);
            SmartDashboard.putNumber("apriltag.tagPitchStrafe", tagPitch);
          } else {
            driveSubsystem.strafeSideways(speed);
          }

        }

        if (myState == MyState.FORWARD) {
          double spinX = driveSubsystem.getSpinPower();
          if (tagPitch < targetPitch) {
            // too close to tag
            speed = -0.1;
          } else {
            speed = 0.1;
          }

          if (tagPitch == null || tagPitch > -12 && tagPitch < -10) {
            driveSubsystem.stopDrive();
            myState = MyState.LAST;
            SmartDashboard.putNumber("apriltag.headingDone", currentHeading);
            SmartDashboard.putNumber("apriltag.tagYawDone", tagYaw);
            SmartDashboard.putNumber("apriltag.tagPitchDone", tagPitch);
            // do nothing
          } else {
            driveSubsystem.autoDrive(0, speed, spinX);
          }
        }
        SmartDashboard.putNumber("apriltag.speed", speed);

        if (myState == MyState.LAST){
          driveSubsystem.setWheelsToStrafe(0);
          //double y = odometrySubsystem.whereIIs.getY();
          if (position == Position.MIDDLE){
            end = true;
          }
          if (position == Position.HUMAN){
            
          }
        }
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.stopDrive();
    myState = MyState.SQUARING;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return end;
  }
}
