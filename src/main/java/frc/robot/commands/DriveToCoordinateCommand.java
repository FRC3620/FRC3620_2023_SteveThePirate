// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Date;

import org.usfirst.frc3620.logger.FastDataLoggerCollections;
import org.usfirst.frc3620.logger.IFastDataLogger;
import org.usfirst.frc3620.misc.PoseOnField;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.OdometrySubsystem;

public class DriveToCoordinateCommand extends CommandBase {
  final PoseOnField destinationPoseOnField;
  final double maxSpeed;
  final double accuracy;
  final double heading;
  final DriveSubsystem driveSubsystem;

  Translation2d destination;
  double distance;

  final boolean doLog = true;
  D2CDataLogger dataLogger;

  /** Creates a new DriveToCoordinateCommand. */
  public DriveToCoordinateCommand(PoseOnField destination, double maxSpeed, double accuracy, double heading, DriveSubsystem driveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.destinationPoseOnField = destination;
    this.maxSpeed = maxSpeed;
    this.accuracy = accuracy;
    this.heading = heading;
    this.driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double adjustedHeading;
    if(DriverStation.getAlliance() == Alliance.Blue){
      adjustedHeading = -heading;
    } else {
      adjustedHeading = heading;
    }

    destination = destinationPoseOnField.getTranslationInMeters();
    driveSubsystem.setAutoSpinMode();
    driveSubsystem.setTargetHeading(adjustedHeading); //RobotContainer.navigationSubsystem.getCorrectedHeading()

    SmartDashboard.putNumber("ourPath.dest_x", destination.getX());
    SmartDashboard.putNumber("ourPath.dest_y", destination.getY());
    SmartDashboard.putNumber("ourPath.dest_heading", heading);
    SmartDashboard.putNumber("ourPath.dest_adjusted_heading", adjustedHeading);

    if (doLog) {
      dataLogger = new D2CDataLogger(getName());
      dataLogger.start();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation2d whereIAm = RobotContainer.odometrySubsystem.getPoseMeters().getTranslation();
    Translation2d ourPath = destination.minus(whereIAm);
    double angle = ourPath.getAngle().getDegrees();

    if(DriverStation.getAlliance() == Alliance.Red){
      angle = 180 - angle;
    } else if(DriverStation.getAlliance() == Alliance.Blue){
      angle = -angle;
    }

    distance = ourPath.getNorm();

    SmartDashboard.putNumber("ourPath.whereIam.x", whereIAm.getX());
    SmartDashboard.putNumber("ourPath.whereIam.y", whereIAm.getY());
    SmartDashboard.putNumber("ourPath.x", ourPath.getX());
    SmartDashboard.putNumber("ourPath.y", ourPath.getY());
    SmartDashboard.putNumber("ourPath.angle", angle);
    SmartDashboard.putNumber("ourPath.distance", distance);

    double spinX = driveSubsystem.getSpinPower();

    double speed = 0;

    if(distance > 1){
      speed = maxSpeed;
    }
    
    if(distance < 1){
      speed = 0.2;
    }

    // need to correct for what direction we are heading
    double desiredAngleRelativeToRobot = angle - RobotContainer.navigationSubsystem.getCorrectedHeading();
    driveSubsystem.autoDrive(desiredAngleRelativeToRobot, speed, spinX);

    if (dataLogger != null) {
      dataLogger.update(whereIAm, ourPath, angle, distance, spinX, speed, desiredAngleRelativeToRobot);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (dataLogger != null) {
      dataLogger.done();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(distance < accuracy){
      return true;
    } else {
      return false;
    }
  }

  @Override
  public boolean runsWhenDisabled(){
    return true;
  }

  class D2CDataLogger extends FastDataLoggerCollections {
    D2CDataLogger(String name) {
      super();
      setInterval(0);
      setFilename(name);
      Date timestamp = new Date();
      setFilenameTimestamp(timestamp);
  
      addMetadata("timestamp", timestamp.toString());
  
      addDataProvider("alliance", () -> DriverStation.getAlliance());
      addDataProvider("destination.x", () -> destinationPoseOnField.getTranslationInMeters().getX());
      addDataProvider("destination.y", () -> destinationPoseOnField.getTranslationInMeters().getY());
      addDataProvider("destination.heading", () -> heading);
      addDataProvider("whereIAm.x", () -> d2c_whereIAm.getX());
      addDataProvider("whereIAm.y", () -> d2c_whereIAm.getY());
      addDataProvider("ourPath.x", () -> d2c_ourPath.getX());
      addDataProvider("ourPath.y", () -> d2c_ourPath.getY());
      addDataProvider("ourPath.angle", () -> d2c_ourPath_angle);
      addDataProvider("ourPath.distance", () -> d2c_ourPath_distance);
      addDataProvider("speed", () -> d2c_speed);
      addDataProvider("spinx", () -> d2c_spinx);
      addDataProvider("robotHeading", () -> RobotContainer.navigationSubsystem.getCorrectedHeading());
      addDataProvider("desiredAngleRelativeToRobot", () -> d2c_desiredAngleRelativeToRobot);
    }

    Translation2d d2c_whereIAm, d2c_ourPath;
    double d2c_ourPath_angle, d2c_ourPath_distance, d2c_spinx, d2c_speed, d2c_desiredAngleRelativeToRobot;

    void update(Translation2d whereIm, Translation2d ourPath, double angle, double distance, double spinX, double speed,
        double desiredAngleRelativeToRobot) {
      d2c_whereIAm = whereIm;
      d2c_ourPath = ourPath;
      d2c_ourPath_angle = angle;
      d2c_ourPath_distance = distance;
      d2c_spinx = spinX;
      d2c_speed = speed;
      d2c_desiredAngleRelativeToRobot = desiredAngleRelativeToRobot;
      update();
    }
  }
}
