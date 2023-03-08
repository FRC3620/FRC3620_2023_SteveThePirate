// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.logger.EventLogging.Level;
import org.usfirst.frc3620.misc.SwerveCalculator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.CannonLocation;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CannonSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.FrontCameraMode;

public class DriveToGamePieceCommand extends CommandBase {
  DriveSubsystem driveSubsystem;
  VisionSubsystem visionSubsystem;
  CannonSubsystem cannonSubsystem;
  double tolerance = 4;
  double lastTimestamp;
  FrontCameraMode currentCameraMode;
  FrontCameraMode pipeline;

  Logger logger = EventLogging.getLogger(getClass(), Level.INFO);

  double targetHeading;
  double startHeading; //when we turn at the end to center on the piece, we use this

  enum MyState {
    SEARCHING, WAITING1, DRIVING, TURNING, PICKUP, STOPPED
  }

  Timer timer;

  MyState myState;

  /** Creates a new DriveToGamePieceCommand. */
  public DriveToGamePieceCommand(FrontCameraMode pipeline, DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, CannonSubsystem cannonSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.visionSubsystem = visionSubsystem;
    this.cannonSubsystem = cannonSubsystem;
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
    logger.info("Searching for {}", pipeline);
    cannonSubsystem.setLocation(CannonLocation.coneFloorPickLocation);
    RobotContainer.flareSubsystem.setColor(Color.kDarkMagenta);
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
    
      double currentHeading = RobotContainer.navigationSubsystem.getCorrectedHeading();

      Double currentTargetYaw = null;
      Double currentTargetPitch = null;
      
      if (target != null) {
        currentTargetYaw = target.getYaw();
        currentTargetPitch = target.getPitch();
        SmartDashboard.putNumber("gamepiece.yaw", currentTargetYaw);
        SmartDashboard.putNumber("gamepiece.pitch", currentTargetPitch);
        if (currentTargetPitch > 14) {
          target = null;
        }
      }

      if (myState == MyState.SEARCHING) {
        if(timer == null){
          timer = new Timer();
          timer.start();
        }
        if(timer.advanceIfElapsed(2)){
          timer = null;
          myState = MyState.WAITING1;
        }
        if (target != null) {
          double spinPower = currentTargetYaw * 0.010;
          spinPower = MathUtil.clamp(spinPower, -.3, .3);

          targetHeading = currentHeading + currentTargetYaw;
          SmartDashboard.putNumber("gamepiece.targetHeading", targetHeading);

          driveSubsystem.autoDrive(currentTargetYaw, 0, spinPower);
          if (Math.abs(SwerveCalculator.calculateAngleDifference(currentHeading, targetHeading)) < tolerance) {
            logger.info ("done searching; targetYaw = {}, currentHeading = {}, targetHeading = {}", currentTargetYaw, currentHeading, targetHeading);
            driveSubsystem.setTargetHeading(targetHeading);
            myState = MyState.WAITING1;
            timer = null;
          }
        } else {
          driveSubsystem.stopDrive();
        }
      } else if (myState == MyState.WAITING1) {
        if (timer == null) {
          timer = new Timer();
          timer.start();
          driveSubsystem.stopDrive();
          driveSubsystem.setWheelsToStrafe(90);
        } else {
          if (timer.hasElapsed(0.5)) {
            if (target != null) {
              myState = MyState.DRIVING;
              timer = null;
              targetHeading = currentHeading + currentTargetYaw;
              logger.info ("done waiting; targetYaw = {}, currentHeading = {}, targetHeading = {}, targetPitch = {}", currentTargetYaw, currentHeading, targetHeading, currentTargetPitch);
            }
          }
        }
      } else if (myState == MyState.DRIVING) {
        cannonSubsystem.setClawPower(0.6);
        double spinPower = driveSubsystem.getSpinPower();
        driveSubsystem.autoDrive(targetHeading - currentHeading, 0.2, spinPower);
        cannonSubsystem.setLocation(CannonLocation.sidewaysConeLocation);
        if (target != null) {
          if (currentTargetPitch < 1.6) { // MAY CHANGE NUM
            logger.info ("done driving; targetPitch = {}", currentTargetPitch);
            myState = MyState.TURNING;
            startHeading = RobotContainer.navigationSubsystem.getCorrectedHeading();
            SmartDashboard.putNumber("gamepiece.startHeading", startHeading);

          }
        }
      }

      if(myState == MyState.TURNING){
        cannonSubsystem.setClawPower(0.6);
        driveSubsystem.autoDrive(startHeading, 0, -0.2);
        if(SwerveCalculator.normalizeAngle(currentHeading - startHeading) < -5){ // we want to be 15 degrees to the left of where we start
          myState = MyState.PICKUP;
        }
      }

      if (myState == MyState.PICKUP) {
        if (timer == null) {
          timer = new Timer();
          timer.start();
        } else {
          driveSubsystem.autoDrive(0, 0.1, 0);
          if (timer.advanceIfElapsed(1.5)) {
            timer = null;
            cannonSubsystem.setClawPower(0.1);
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
    RobotContainer.flareSubsystem.setColor(Color.kBlack);
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
