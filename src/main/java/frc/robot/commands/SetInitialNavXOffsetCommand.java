// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.INavigationSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetInitialNavXOffsetCommand extends InstantCommand {
  INavigationSubsystem navigationSubsystem;
  DriveSubsystem m_DriveSubsystem;
  double navXOffsetAngle;
  public SetInitialNavXOffsetCommand(INavigationSubsystem navigationSubsystem, DriveSubsystem m_DriveSubsystem, double offsetAngle) {
    super();
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    this.navigationSubsystem = navigationSubsystem;
    this.m_DriveSubsystem = m_DriveSubsystem;
    navXOffsetAngle = offsetAngle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    navigationSubsystem.setHeadingOffset(navXOffsetAngle);
    navigationSubsystem.reset();
    m_DriveSubsystem.setTargetHeading(navigationSubsystem.getCorrectedHeading());
    
  }
}