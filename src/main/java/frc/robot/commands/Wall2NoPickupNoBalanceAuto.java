// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
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

/**
 * Start/place at wall, go out and grab piece via DriveToGamePiece, place piece
 */
public class Wall2NoPickupNoBalanceAuto extends SequentialCommandGroup {
  /** Creates a new Human2NoBalanceAuto. */
  public Wall2NoPickupNoBalanceAuto(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, CannonSubsystem cannonSubsystem, OdometrySubsystem odometrySubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetInitialNavXOffsetCommand(RobotContainer.navigationSubsystem, driveSubsystem, 180)
      ,
      // tell odometry where we is
      new ZapOdometryCommand(FieldLocation.wallStart)
      ,
      new InstantCommand(() -> visionSubsystem.setFrontCameraMode(FrontCameraMode.CUBES))
      ,
      new SetCannonLocationCommand(CannonLocation.coneHighLocation)
      ,
      new WaitCommand(4)
      ,
      new CannonClawOutCommand(cannonSubsystem, -0.8).withTimeout(1.5)
      ,
      new InstantCommand(() -> driveSubsystem.setWheelsToStrafe(90))
      ,
      new SetCannonLocationCommand(CannonLocation.parkLocation)
      ,
      new DriveToCoordinateCommand(FieldLocation.wallHalfway, 0.6, 0.22, 180, driveSubsystem)
      ,
      new SetCannonLocationCommand(CannonLocation.lowLocation)
      ,
      new DriveToCoordinateCommand(FieldLocation.wallMiddle, 0.4, 0.22, 0, driveSubsystem)
      ,
      // set cannon down to gamepiece somewhere
      new DriveToGamePieceCommand(FrontCameraMode.CUBES, driveSubsystem, visionSubsystem, cannonSubsystem)
      ,
      new InstantCommand(() -> visionSubsystem.setFrontCameraMode(FrontCameraMode.APRILTAGS))
      ,
      new SetCannonLocationCommand(CannonLocation.parkLocation)
      ,
      new DriveToCoordinateCommand(FieldLocation.wallMiddle, 0.4, 0.22, 0, driveSubsystem)
      ,
      new DriveToCoordinateCommand(FieldLocation.wallHalfway, 0.4, 0.22, 180, driveSubsystem)
      ,
      new DriveToCoordinateCommand(FieldLocation.wallCommunity, 0.2, 0.1, 180, driveSubsystem)
      ,
      new DriveToAprilTagCommand(1, Position.MIDDLE, driveSubsystem, visionSubsystem, odometrySubsystem)
      ,
      new SetCannonLocationCommand(CannonLocation.cubeHighLocation)
      ,
      new WaitCommand(1.6)
      ,
      new CannonClawOutCommand(cannonSubsystem, -0.8).withTimeout(1.5)
    );
  }
}
