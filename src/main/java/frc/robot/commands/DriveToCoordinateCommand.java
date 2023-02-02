// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.print.attribute.standard.Destination;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.OdometrySubsystem;

public class DriveToCoordinateCommand extends CommandBase {
  Translation2d destination;
  double distance;
  DriveSubsystem driveSubsystem;
  /** Creates a new DriveToCoordinateCommand. */
  public DriveToCoordinateCommand(Translation2d destination, DriveSubsystem driveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.destination = destination;
    this.driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveSubsystem.setAutoSpinMode();
    driveSubsystem.setTargetHeading(RobotContainer.navigationSubsystem.getCorrectedHeading());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation2d whereIAm = OdometrySubsystem.getPoseMeters().getTranslation();
    Translation2d ourPath = destination.minus(whereIAm);
    double angle = ourPath.getAngle().getDegrees();
    
    if(DriverStation.getAlliance() == Alliance.Red){
      angle = 180 - angle;
    } else if(DriverStation.getAlliance() == Alliance.Blue){
      angle = -angle;
    }

    distance = ourPath.getNorm();

    SmartDashboard.putNumber("ourPath.x", ourPath.getX());
    SmartDashboard.putNumber("ourPath.y", ourPath.getY());
    SmartDashboard.putNumber("ourPath.angle", angle);
    SmartDashboard.putNumber("ourPath.distance", distance);

    double spinX = driveSubsystem.getSpinPower();

    double speed = 0;

    if(distance > 1){
      speed = 0.5;
    }
    
    if(distance < 1){
      speed = 0.1;
    }

    // need to correct for what direction we are heading
    double desiredAngleRelativeToRobot = angle - RobotContainer.navigationSubsystem.getCorrectedHeading();
    driveSubsystem.autoDrive(desiredAngleRelativeToRobot, speed, spinX);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(distance < .2){
      return true;
    } else {
      return false;
    }
  }

  @Override
  public boolean runsWhenDisabled(){
    return true;
  }
}
