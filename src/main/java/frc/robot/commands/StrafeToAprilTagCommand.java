package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.VisionSubsystem;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class StrafeToAprilTagCommand extends CommandBase {
  boolean end = false;
  DriveSubsystem driveSubsystem;
  VisionSubsystem visionSubsystem;
  double lastTimestamp;

  public StrafeToAprilTagCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
    this.driveSubsystem = RobotContainer.driveSubsystem;
    this.visionSubsystem = RobotContainer.visionSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // set the wheels to strafe here
    driveSubsystem.setWheelsToStrafe(0);
    end = false;
    lastTimestamp = -1;
    // driveSubsystem.setWheelsToStrafe(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    PhotonPipelineResult result = visionSubsystem.getLastFrontCameraAprilTagsResult(lastTimestamp);
    if (result != null) {
      lastTimestamp = result.getTimestampSeconds();
      PhotonTrackedTarget target = VisionSubsystem.getTargetById(result, 1);
      Double distanceToX = target.getYaw(); //this is something random, idk what it actually is
      double targetSpeed = 0.0;

      if (distanceToX != null) {
        targetSpeed = (1.2 * (distanceToX - 0.5));
      } else {
        targetSpeed = 0;
      }

      if (targetSpeed > -0.2 && targetSpeed < 0.2) {
        if (targetSpeed < 0) {
          targetSpeed = -0.1;
        } else {
          targetSpeed = 0.1;
        }
      }

    // watch the target, and call strafeWidways as needed
    if (distanceToX == null || distanceToX > 0.46 && distanceToX < 0.54) {
      driveSubsystem.stopDrive();
      if (distanceToX > 0.45 && distanceToX < 0.55) {
        end = true;
      }
      // do nothing
    } else {
      driveSubsystem.strafeSideways(targetSpeed);
    }
    // right
    /*
     * else if(distanceToX > 0.25)
     * {
     * driveSubsystem.strafeSideways(0.2);
     * 
     * }
     * //left
     * else
     * {
     * driveSubsystem.strafeSideways(-0.2);
     * }
     */
    // driveSubsystem.strafeSideways(0);
    SmartDashboard.putNumber("targetSpeed", targetSpeed);
     }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // make sure the drive is stopped here!
    driveSubsystem.stopDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return end;
  }
}
// :D