// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CannonExtendMechanism;
import frc.robot.subsystems.CannonSubsystem;

public class CannonExtendCommand extends CommandBase {
  /** Creates a new CannonExtendCommand. */
  CannonSubsystem cannonSubsystem;
  double desiredLength;
  /**
   * 
   * Creates a new MoveTurretCommand.
   */
  public CannonExtendCommand(CannonSubsystem _subsystem, double _desiredLength) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(_subsystem);
    cannonSubsystem = _subsystem;
    desiredLength = _desiredLength;
  } 

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    cannonSubsystem.setLength(desiredLength);
  }

  // Called every time the scheduler runs while the command is scheduled.

  @Override
  public void execute() {
   
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}