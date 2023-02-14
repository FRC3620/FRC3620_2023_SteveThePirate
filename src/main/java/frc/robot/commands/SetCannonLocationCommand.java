// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.CannonLocation;
import frc.robot.RobotContainer;

public class SetCannonLocationCommand extends InstantCommand {
  CannonLocation cannonLocation;
  /** Creates a new SetCannonLocationCommand. */
  public SetCannonLocationCommand(CannonLocation cannonLocation) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.cannonSubsystem);
    this.cannonLocation = cannonLocation;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.cannonSubsystem.setLocation(cannonLocation);;
  }
}
