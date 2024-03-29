// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class XModeForThreeSecondsCommand extends CommandBase {
  Timer timer;
  DriveSubsystem driveSubsystem;
  /** Creates a new XModeForThreeSecondsCommand. */
  public XModeForThreeSecondsCommand(DriveSubsystem driveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(timer == null){
      timer = new Timer();
      timer.start();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSubsystem.xMode();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer = null;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(timer.advanceIfElapsed(3)){
      return true;
    }
    return false;
  }
}
