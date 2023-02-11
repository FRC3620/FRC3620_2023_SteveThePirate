// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.usfirst.frc3620.misc.PoseOnField;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SimpleTestAuto extends SequentialCommandGroup {
  PoseOnField fred = PoseOnField.fromRedAlliancePositionInMeters(13.7, 4.7);
  PoseOnField bigStu = PoseOnField.fromRedAlliancePositionInMeters(10.8, 5);
  
  //to be replaced by drive to w/ vision
  PoseOnField gamePiece = PoseOnField.fromRedAlliancePositionInMeters(9.3, 3.9);
  PoseOnField placePosition = PoseOnField.fromRedAlliancePositionInMeters(14.6, 4.8);

  /** Creates a new SimpleTestAuto. */
  public SimpleTestAuto(DriveSubsystem driveSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetInitialNavXOffsetCommand(RobotContainer.navigationSubsystem, driveSubsystem, 180)
      ,
      new WaitForSaneOdometryCommand()
      ,
      new DriveToCoordinateCommand(placePosition, 0.2, 0.2, 180, driveSubsystem)
      ,
      new DriveToCoordinateCommand(bigStu, .3, .2, -45, driveSubsystem)
      ,
      new DriveToCoordinateCommand(gamePiece, .4, .2, -45, driveSubsystem)
      ,
      new DriveToCoordinateCommand(bigStu, .4, .1, 180, driveSubsystem)
      ,
      new DriveToCoordinateCommand(placePosition, .3, .2, 180, driveSubsystem)
    );
  }
}
