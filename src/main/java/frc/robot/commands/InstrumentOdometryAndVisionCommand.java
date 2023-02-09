// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class InstrumentOdometryAndVisionCommand extends CommandBase {
  /** Creates a new InstrumentOdometryAndVisionCommand. */
  public InstrumentOdometryAndVisionCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.odometrySubsystem.makeBlindOdometry(DriverStation.getAlliance());
    RobotContainer.visionSubsystem.startVisionDataLogger();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.visionSubsystem.doneWithVisionDataLogger();
    RobotContainer.odometrySubsystem.doneWithBlindOdometry();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
