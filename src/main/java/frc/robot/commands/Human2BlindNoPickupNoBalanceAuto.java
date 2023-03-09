// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
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
public class Human2BlindNoPickupNoBalanceAuto extends SequentialCommandGroup {
  /** Creates a new Human2NoBalanceAuto. */
  public Human2BlindNoPickupNoBalanceAuto(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, CannonSubsystem cannonSubsystem, OdometrySubsystem odometrySubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetInitialNavXOffsetCommand(RobotContainer.navigationSubsystem, driveSubsystem, 180)
      ,
      // tell odometry where we is
      new ZapOdometryCommand(FieldLocation.humanStart)
      ,
      new InstantCommand(() -> visionSubsystem.setFrontCameraMode(FrontCameraMode.CUBES))
      ,
      new SetCannonLocationCommand(CannonLocation.coneHighLocation)
      ,
      new WaitCommand(2) // change later??
      ,
      new CannonClawOutCommand(cannonSubsystem, -0.8).withTimeout(.5)
      ,
      new InstantCommand(() -> driveSubsystem.setWheelsToStrafe(90))
      ,
      new SetCannonLocationCommand(CannonLocation.parkLocation)
      ,
      new WaitCommand(.5)
      ,
      new DriveToCoordinateCommand(FieldLocation.humanHalfway, 0.6, 0.21, 180, driveSubsystem) // speed = 0.2
      ,
      new SetCannonLocationCommand(CannonLocation.lowLocation)
      ,
      new DriveToCoordinateCommand(FieldLocation.humanMiddleBlind, 0.2, 0.2, -30, driveSubsystem)
      ,
      new SetCannonLocationCommand(CannonLocation.sidewaysConeLocation)
      ,
      new ParallelDeadlineGroup(
        new DriveToCoordinateCommand(FieldLocation.humanBlindPosition, .2, 0.1, 0, driveSubsystem)
        ,
        new CannonClawInCommand(cannonSubsystem, 0.6)
      )
      ,
      new InstantCommand(() -> visionSubsystem.setFrontCameraMode(FrontCameraMode.APRILTAGS))
      ,
      // should we do this or go to the position for leveling?
      new SetCannonLocationCommand(CannonLocation.parkLocation)
      ,
      new SetCannonLocationCommand(CannonLocation.parkLocation)
      ,
      new DriveToCoordinateCommand(FieldLocation.humanMiddle, 0.4, 0.21, 180, driveSubsystem)
      ,
      new DriveToCoordinateCommand(FieldLocation.humanHalfway, 0.4, 0.22, 180, driveSubsystem)
      ,
      new DriveToCoordinateCommand(FieldLocation.humanCommunity, 0.2, 0.1, 180, driveSubsystem)
      ,
      new DriveToAprilTagCommand(3, Position.MIDDLE, driveSubsystem, visionSubsystem, odometrySubsystem)
      ,
      new SetCannonLocationCommand(CannonLocation.cubeHighLocation)
      ,
      new WaitCommand(1.5)
      ,
      new CannonClawOutCommand(cannonSubsystem, -0.8).withTimeout(.5)
    );
  }
}
