// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestCoordinateAuto extends SequentialCommandGroup {
  Translation2d fred = new Translation2d(13.6, 4.6);
  Translation2d bigStu = new Translation2d(10.8, 4.7);
  Translation2d dougyBoi = new Translation2d(9.4, 2.0);
  /** Creates a new TestCoordinateAuto. */
  public TestCoordinateAuto(DriveSubsystem driveSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetInitialNavXOffsetCommand(RobotContainer.navigationSubsystem, driveSubsystem, 180)
      ,
      new WaitForAccurateOdometryCommand(fred)
      ,
      new DriveToCoordinateCommand(bigStu, 0.5, 1, driveSubsystem)
      ,
      new DriveToCoordinateCommand(dougyBoi, 0.5, 0.2, driveSubsystem)
      ,
      new DriveToCoordinateCommand(bigStu, 0.5, 0.5, driveSubsystem)
      ,
      new DriveToCoordinateCommand(fred, 0.5, 0.2, driveSubsystem)
    );

  }
}
