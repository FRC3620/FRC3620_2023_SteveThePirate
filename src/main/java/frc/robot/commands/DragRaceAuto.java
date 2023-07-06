// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.FieldLocation;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DragRaceAuto extends SequentialCommandGroup {
  /** Creates a new DragRaceAuto. */
  public DragRaceAuto(DriveSubsystem driveSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetInitialNavXOffsetCommand(RobotContainer.navigationSubsystem, driveSubsystem, 90)
      ,
      new InstantCommand(() -> driveSubsystem.fixRelativeEncoders())
      ,
      new InstantCommand(() -> driveSubsystem.setWheelsToStrafe(90))
      ,
      new ZapOdometryCommand(FieldLocation.dragRaceStart)
      ,
      new DriveToCoordinateCommand(FieldLocation.dragRaceFinish, 1, 0.15, 90, driveSubsystem) //was .6 speed
    );
  }
}
