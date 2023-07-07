// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.FieldLocation;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DoubleDragRaceAuto extends SequentialCommandGroup {
  /** Creates a new DoubleDragRaceAuto. */
  public DoubleDragRaceAuto(DriveSubsystem driveSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetInitialNavXOffsetCommand(RobotContainer.navigationSubsystem, driveSubsystem, 0)
      ,
      new InstantCommand(() -> driveSubsystem.fixRelativeEncoders())
      ,
      new InstantCommand(() -> driveSubsystem.setWheelsToStrafe(90))
      ,
      new ZapOdometryCommand(FieldLocation.doubleDrag1)
      ,
      new WaitCommand(1)
      ,
      new DriveToCoordinateCommand(FieldLocation.doubleDrag2, 1, 0.15, 0, driveSubsystem)
      ,
      new WaitCommand(1)
      ,
      new DriveToCoordinateCommand(FieldLocation.doubleDrag1, 1, .15, 0, driveSubsystem)
      ,
      new InstantCommand(() -> driveSubsystem.setWheelsToStrafe(0))
      ,
      new WaitCommand(1)
      ,
      new DriveToCoordinateCommand(FieldLocation.doubleDrag3, 1, .15, 0, driveSubsystem)
      ,
      new WaitCommand(1)
      ,
      new DriveToCoordinateCommand(FieldLocation.doubleDrag1, 1, .15, 0, driveSubsystem)
    );
  }
}
