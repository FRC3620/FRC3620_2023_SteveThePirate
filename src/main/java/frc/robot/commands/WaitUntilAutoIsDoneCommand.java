// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class WaitUntilAutoIsDoneCommand extends CommandBase {
  double howLongSeconds;
  /** Creates a new WaitUntilAutoIsDoneCommand. */
  public WaitUntilAutoIsDoneCommand(double howLongSeconds) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.howLongSeconds = howLongSeconds;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double matchTime = DriverStation.getMatchTime();
    if (matchTime >= 0 && matchTime < howLongSeconds) {
      return true;
    } else {
      return false;
    }
  }
}
