package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystem.Corner;

import org.usfirst.frc3620.misc.IAutonomousLogger;

public class AutoDriveCommand extends CommandBase {

  private DriveSubsystem driveSubsystem;

  private double initialPositionRightFront;
  private double initialPositionLeftFront;
  private double initialPositionRightBack;
  private double initialPositionLeftBack;
  private double distanceTravelled;
  private double desiredDistance;
  private double desiredAngle;
  private double desiredHeading;
  private double pathSpeed;

  private Timer timer;

  private IAutonomousLogger autonomousLogger;
  private String legName;

  public AutoDriveCommand(double distance, double strafeAngle, double speed, double heading,
      DriveSubsystem driveSubsystem) {
    this(distance, strafeAngle, speed, heading, driveSubsystem, null, null);
  }

  public AutoDriveCommand(double distance, double strafeAngle, double speed, double heading,
      DriveSubsystem driveSubsystem, String legName, IAutonomousLogger autonomousLogger) {
    this.driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);

    desiredDistance = distance;
    desiredAngle = strafeAngle;
    desiredHeading = heading;
    pathSpeed = speed;

    this.legName = legName;
    this.autonomousLogger = autonomousLogger;

    this.timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialPositionRightFront = driveSubsystem.getCornerDrivePosition(Corner.RF);
    // drive motor
    initialPositionLeftFront = driveSubsystem.getCornerDrivePosition(Corner.LF);
    initialPositionRightBack = driveSubsystem.getCornerDrivePosition(Corner.RB);
    initialPositionLeftBack = driveSubsystem.getCornerDrivePosition(Corner.LB);
    if (autonomousLogger != null) {
      if (legName == null) {
        autonomousLogger.setLegName(getClass().getName());
      } else {
        autonomousLogger.setLegName(legName);
      }
      autonomousLogger.setInitialDrivePositions(initialPositionLeftFront, initialPositionRightFront,
          initialPositionLeftBack, initialPositionRightBack);
      autonomousLogger.setCurrentDrivePositions(initialPositionLeftFront, initialPositionRightFront,
          initialPositionLeftBack, initialPositionRightBack);
      autonomousLogger.setElapsed(0.0);
      autonomousLogger.doLog();
      timer.reset();
      timer.start();
    }
    driveSubsystem.setAutoSpinMode();
    driveSubsystem.setTargetHeading(desiredHeading);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double spinX = driveSubsystem.getSpinPower();

    // need to correct for what direction we are heading
    double desiredAngleRelativeToRobot = desiredAngle - RobotContainer.navigationSubsystem.getCorrectedHeading();
    driveSubsystem.autoDrive(desiredAngleRelativeToRobot, pathSpeed, spinX);

    double currentPositionRightFront = driveSubsystem.getCornerDrivePosition(Corner.RF);
    double currentPositionLeftFront = driveSubsystem.getCornerDrivePosition(Corner.LF);
    double currentPositionRightBack = driveSubsystem.getCornerDrivePosition(Corner.RB);
    double currentPositionLeftBack = driveSubsystem.getCornerDrivePosition(Corner.LB);

    double distanceTravelledRightFront = Math.abs(currentPositionRightFront - initialPositionRightFront);
    double distanceTravelledLeftFront = Math.abs(currentPositionLeftFront - initialPositionLeftFront);
    double distanceTravelledRightBack = Math.abs(currentPositionRightBack - initialPositionRightBack);
    double distanceTravelledLeftBack = Math.abs(currentPositionLeftBack - initialPositionLeftBack);

    distanceTravelled = (distanceTravelledRightFront + distanceTravelledLeftFront + distanceTravelledRightBack
        + distanceTravelledLeftBack) / 4;

    if (autonomousLogger != null) {
      autonomousLogger.setCurrentDrivePositions(currentPositionLeftFront, currentPositionRightFront,
          currentPositionLeftBack, currentPositionRightBack);
      autonomousLogger.setElapsed(timer.get());
      autonomousLogger.doLog();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.teleOpDrive(0, 0, 0);
    if (autonomousLogger != null) {
      timer.stop();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (distanceTravelled >= desiredDistance) {
      return true;
    }
    return false;
  }
}