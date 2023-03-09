package frc.robot.commands;

import java.text.DecimalFormat;

import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.logger.EventLogging.Level;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.usfirst.frc3620.misc.SwerveCalculator;

import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

public class AutoSpinCommand extends CommandBase {

  private DriveSubsystem driveSubsystem;

  Logger logger = EventLogging.getLogger(getClass(), Level.INFO);

  private double desiredHeading;
  private double pathSpeed;

  public AutoSpinCommand(double speed, double heading, DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);

    desiredHeading = heading;
    pathSpeed = speed;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Translation2d whereIAm = RobotContainer.odometrySubsystem.getPoseMeters().getTranslation();
    double heading = RobotContainer.navigationSubsystem.getCorrectedHeading();
    logger.info ("starting at @ {}, {};  robot heading = {}",
      f2(whereIAm.getX()), f2(whereIAm.getY()), heading);
    driveSubsystem.setForcedManualModeTrue();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //double heading = driveSubsystem.getNavXFixedAngle(); 
    double spinX = pathSpeed;
    driveSubsystem.autoDrive(0, 0, spinX);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Translation2d whereIAm = RobotContainer.odometrySubsystem.getPoseMeters().getTranslation();
    double heading = RobotContainer.navigationSubsystem.getCorrectedHeading();
    logger.info ("done @ {}, {};  robot heading = {}, interrupted = {}",
      f2(whereIAm.getX()), f2(whereIAm.getY()), heading, interrupted);
    driveSubsystem.teleOpDrive(0,0,0);
    driveSubsystem.setForcedManualModeFalse();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double diff = SwerveCalculator.calculateAngleDifference(RobotContainer.navigationSubsystem.getCorrectedHeading(), desiredHeading);
    if(Math.abs(diff) < 5){
      return true;
    }
    return false;
  }

  DecimalFormat f2format = new DecimalFormat("#.##");

  String f2 (double d) {
    return f2format.format(d);
  }
}


