package frc.robot.commands;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class DriveToAprilTagCommand extends CommandBase {
  boolean end = false;
  boolean iAmDriving = false;
  DriveSubsystem driveSubsystem;
  VisionSubsystem visionSubsystem;
  AutoDriveCommand aprilTagDrive;

  boolean joemama = true;

  public DriveToAprilTagCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.visionSubsystem = visionSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // set the wheels to strafe here
    visionSubsystem.clearTag1Transform();
    iAmDriving = false;
    end = false;
    // driveSubsystem.setWheelsToStrafe(0);
  }

  // AutoDriveCommand autoDriveCommand;

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double distance;
    double angle;
    double maxSpeed = 0.5;
    double minSpeed = 0.15;
    double speedX;
    double speedZ;
    double offset = 36;

    if (joemama) {
      Transform3d cameraPosition = visionSubsystem.getCameraToTag();
      if (cameraPosition != null) {
        double cameraPositionX = cameraPosition.getX();
        double cameraPositionY = cameraPosition.getY();
        double cameraPositionZ = cameraPosition.getZ();

        double cameraPositionXinch = cameraPositionX * 39.3701;
        double cameraPositionYinch = cameraPositionY * 39.3701;
        double cameraPositionZinch = cameraPositionZ * 39.3701;

        SmartDashboard.putNumber("DTAT.Xinch", cameraPositionXinch);
        SmartDashboard.putNumber("DTAT.Yinch", cameraPositionYinch);
        SmartDashboard.putNumber("DTAT.Zinch", cameraPositionZinch);
        distance = Math.sqrt(Math.pow(cameraPositionZinch - offset, 2) + Math.pow(cameraPositionXinch, 2));
        angle = Math.toDegrees(Math.atan(cameraPositionXinch / cameraPositionZinch));
        SmartDashboard.putNumber("DTAT.distance", distance);
        SmartDashboard.putNumber("DTAT.Strafe Angle", angle);

        // iAmDriving = true;
        // autoDriveCommand = new AutoDriveCommand(distance, angle, speed, 0,
        // driveSubsystem);
        // autoDriveCommand.initialize();

        speedX = cameraPositionXinch / 100;
        speedZ = Math.abs(cameraPositionZinch / 240);

        if (Math.abs(cameraPositionXinch) < 3.1) {
          speedX = 0;
        }

        if (Math.abs(speedX) > maxSpeed) {
          if (speedX < 0) {
            speedX = -maxSpeed;
          } else {
            speedX = maxSpeed;
          }
        } else if (Math.abs(speedX) < minSpeed) {
          if (speedX < 0) {
            speedX = -minSpeed;
          } else {
            speedX = minSpeed;
          }
        }

        if (speedZ > maxSpeed) {
          speedZ = maxSpeed;
        } else if (speedZ < minSpeed) {
          speedZ = minSpeed;
        }

        if (distance > 36) {
          driveSubsystem.teleOpDrive(speedX, speedZ, 0);
          SmartDashboard.putNumber("speedX", speedX);
          SmartDashboard.putNumber("speedZ", speedZ);
        } else {
          SmartDashboard.putNumber("speedX", 0);
          SmartDashboard.putNumber("speedZ", 0);
          driveSubsystem.stopDrive();
        }
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // make sure the drive is stopped here!
    driveSubsystem.stopDrive();
    // driveSubsystem.stopDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    /*
     * if(distance < 36)
     * {
     * return true;
     * }
     */
    return false;
  }
}

// :D