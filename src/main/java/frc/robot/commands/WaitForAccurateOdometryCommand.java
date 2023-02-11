// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.usfirst.frc3620.misc.PoseOnField;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.OdometrySubsystem;

public class WaitForAccurateOdometryCommand extends CommandBase {
  PoseOnField destinationPoseOnField;
  double distance;
  Translation2d desiredCoord;
  /** Creates a new WaitForSaneOdometryCommand. */
  public WaitForAccurateOdometryCommand(PoseOnField destinationPoseOnField) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.destinationPoseOnField = destinationPoseOnField;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    desiredCoord = destinationPoseOnField.getTranslationInMeters();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation2d whereIAm = RobotContainer.odometrySubsystem.getPoseMeters().getTranslation();
    Translation2d ourPath = desiredCoord.minus(whereIAm);

    distance = ourPath.getNorm();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(distance > 1.5){
      return false;
    } else {
      return true;
    }
  }
}
