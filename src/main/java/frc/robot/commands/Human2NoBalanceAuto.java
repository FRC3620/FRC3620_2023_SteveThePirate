// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.CannonLocation;
import frc.robot.FieldLocation;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveToAprilTagCommand.Position;
import frc.robot.subsystems.CannonSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.OdometrySubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.FrontCameraMode;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Human2NoBalanceAuto extends SequentialCommandGroup {
  /** Creates a new Human2NoBalanceAuto. */
  public Human2NoBalanceAuto(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, CannonSubsystem cannonSubsystem, OdometrySubsystem odometrySubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetInitialNavXOffsetCommand(RobotContainer.navigationSubsystem, driveSubsystem, 180)
      ,
      new WaitForSaneOdometryCommand()
      ,
      new DriveToAprilTagCommand(3, Position.HUMAN, driveSubsystem, visionSubsystem, odometrySubsystem)
      ,
      new SetCannonLocationCommand(CannonLocation.coneHighLocation)
      ,
      new SetCannonClawSpeedCommand(cannonSubsystem, -0.8)
      ,
      new WaitCommand(1)
      ,
      new SetCannonClawSpeedCommand(cannonSubsystem, 0)
      ,
      new SetCannonLocationCommand(CannonLocation.parkLocation)
      ,
      new DriveToCoordinateCommand(FieldLocation.humanMiddle, 0.2, 0.1, 0, driveSubsystem)
      ,
      // set cannon down to gamepiece somewhere
      new DriveToGamePieceCommand(FrontCameraMode.CUBES, driveSubsystem, visionSubsystem)
      ,
      new SetCannonLocationCommand(CannonLocation.parkLocation)
      ,
      new DriveToCoordinateCommand(FieldLocation.humanMiddle, 0.2, 0.1, 180, driveSubsystem)
      ,
      new DriveToCoordinateCommand(FieldLocation.humanStart, 0.2, 0.1, 180, driveSubsystem)
      ,
      new DriveToAprilTagCommand(3, Position.MIDDLE, driveSubsystem, visionSubsystem, odometrySubsystem)
      ,
      new SetCannonLocationCommand(CannonLocation.coneHighLocation)
      ,
      new SetCannonClawSpeedCommand(cannonSubsystem, -0.8)
      ,
      new WaitCommand(1)
      ,
      new SetCannonClawSpeedCommand(cannonSubsystem, 0)
      ,
      new SetCannonLocationCommand(CannonLocation.parkLocation)
    );
  }
}
