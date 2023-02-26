// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.usfirst.frc3620.logger.IFastDataLogger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.DrivingDataLogger;
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
  double staticYInLast;
  Timer timer;

  enum MyState {
    SQUARING, SET_TO_STRAFE, STRAFING, SET_TO_FORWARD, FORWARD, LAST
  }

  MyState myState;

  public enum Position {
    MIDDLE, WALL, HUMAN
  }

  Position position;

  boolean doLogging = true;
  IFastDataLogger drivingDataLogger = null;

  /** Creates a new DriveToAprilTagCommand. */
  public DriveToAprilTagCommand(int tagID, Position position, DriveSubsystem driveSubsystem,
      VisionSubsystem visionSubsystem, OdometrySubsystem odometrySubsystem) {
    this.tagID = tagID;
    this.position = position;
    // TODO why are we going to RobotContainer for these? they are available as parameters.
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

    if (doLogging) {
      setupDrivingDataLogger();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d whereIIs = odometrySubsystem.getPoseMeters();
    double currentHeading = RobotContainer.navigationSubsystem.getCorrectedHeading();
  
    dl_tagYaw = null;
    dl_tagPitch = null;
    dl_tagId = null;
    dl_strafePower = null;

    if (myState == MyState.SQUARING) {
      driveSubsystem.setTargetHeading(180);
      double spinX = 3 * driveSubsystem.getSpinPower();
      driveSubsystem.autoDrive(0, 0, spinX);
      if (Math.abs(currentHeading) > 178 && Math.abs(currentHeading) < 182) {
        driveSubsystem.stopDrive();
        myState = MyState.SET_TO_STRAFE;
        SmartDashboard.putNumber("apriltag.headingSquare", currentHeading);
      }
    }
    if (myState == MyState.SET_TO_STRAFE) {
      if (timer == null) {
        timer = new Timer();
        timer.start();
      }
      driveSubsystem.setWheelsToStrafe(0);
      if (timer.advanceIfElapsed(.25)) {
        myState = MyState.STRAFING;
        timer = null;
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

        dl_tagYaw = tagYaw;
        dl_tagPitch = tagPitch;
        dl_tagId = target.getFiducialId();
        double targetYaw = 10.38;
        double targetPitch = 10.9;

        double targetYawTolerance = 0.7;
        double targetPitchTolerance = 0.9;

        double speed = 0.0;

        SmartDashboard.putString("apriltag.state", myState.toString());

        if (myState == MyState.STRAFING) {

          if (tagYaw < targetYaw) {
            speed = -0.1;
          } else {
            speed = 0.1;
          }

          if (tagYaw == null || tagYaw > targetYaw - targetYawTolerance && tagYaw < targetYaw + targetYawTolerance) {
            driveSubsystem.stopDrive();
            myState = MyState.SET_TO_FORWARD;
            SmartDashboard.putNumber("apriltag.headingStrafe", currentHeading);
            SmartDashboard.putNumber("apriltag.tagYawStrafe", tagYaw);
            SmartDashboard.putNumber("apriltag.tagPitchStrafe", tagPitch);
          } else {
            driveSubsystem.strafeSideways(speed);
          }

        }

        if (myState == MyState.SET_TO_FORWARD) {
          if (timer == null) {
            timer = new Timer();
            timer.start();
          }
          driveSubsystem.setWheelsToStrafe(90);
          if (timer.advanceIfElapsed(.25)) {
            myState = MyState.FORWARD;
            timer = null;
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

          if (tagPitch == null || tagPitch > targetPitch - targetPitchTolerance && tagPitch < targetPitch + targetPitchTolerance) {
            driveSubsystem.stopDrive();
            staticYInLast = whereIIs.getY();
            SmartDashboard.putNumber("static y", staticYInLast);
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

        if (myState == MyState.LAST) {
          double strafeDistance = 0.3048; //change this
          double y = whereIIs.getY();
          double power = 0.1;

          driveSubsystem.setWheelsToStrafe(0);
          if (position == Position.MIDDLE) {
            end = true;
          }
          if (DriverStation.getAlliance() == Alliance.Blue) {
            power = -0.1;
          }

          if (position == Position.HUMAN){
            dl_strafePower = -power;
            driveSubsystem.autoDrive(90, -power, 0);
            if(y > staticYInLast + strafeDistance){
              dl_strafePower = 0.0;
              driveSubsystem.stopDrive();
              end = true;
            }
          }

          if (position == Position.WALL){
            dl_strafePower = power;
            driveSubsystem.autoDrive(90, power, 0);
            if(y < staticYInLast - strafeDistance){
              dl_strafePower = 0.0;
              driveSubsystem.stopDrive();
              end = true;
            }
          }
        }
      }
    }

    if (drivingDataLogger != null) drivingDataLogger.update();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (drivingDataLogger != null) {
      drivingDataLogger.done();
      drivingDataLogger = null;
    }
    driveSubsystem.stopDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return end;
  }

  Double dl_tagYaw, dl_tagPitch, dl_strafePower;
  Integer dl_tagId;
  
  void setupDrivingDataLogger() {
    drivingDataLogger = DrivingDataLogger.getShootingDataLogger("drive_to_apriltag");
    drivingDataLogger.setInterval(0.0);
    drivingDataLogger.addMetadata("position", position.toString());
    drivingDataLogger.addDataProvider("state", () -> myState.toString());
    drivingDataLogger.addDataProvider("state_ord", () -> myState.ordinal());
    drivingDataLogger.addDataProvider("static_y_in_last", () -> staticYInLast);
    drivingDataLogger.addDataProvider("strafe_power", () -> dl_strafePower);

    drivingDataLogger.addDataProvider("tagYaw", () -> dl_tagYaw);
    drivingDataLogger.addDataProvider("tagPitch", () -> dl_tagPitch);
    drivingDataLogger.addDataProvider("tagId", () -> dl_tagId);

    drivingDataLogger.start();
  }
}
