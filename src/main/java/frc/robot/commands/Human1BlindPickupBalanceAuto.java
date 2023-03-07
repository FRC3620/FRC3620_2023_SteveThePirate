// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.usfirst.frc3620.misc.PoseOnField;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.CannonLocation;
import frc.robot.FieldLocation;
import frc.robot.RobotContainer;
import frc.robot.TargetPoseOnField;
import frc.robot.commands.DriveToAprilTagCommand.Position;
import frc.robot.subsystems.CannonSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.OdometrySubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.FrontCameraMode;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Human1BlindPickupBalanceAuto extends SequentialCommandGroup {
  final DriveSubsystem driveSubsystem;
  /** Creates a new Mid1BalanceAuto. */
  public Human1BlindPickupBalanceAuto(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, CannonSubsystem cannonSubsystem, OdometrySubsystem odometrySubsystem) {
    this.driveSubsystem = driveSubsystem;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetInitialNavXOffsetCommand(RobotContainer.navigationSubsystem, driveSubsystem, 180)
      ,
      // tell odometry where we is
      new ZapOdometryCommand(FieldLocation.humanStart)
      ,
      new SetCannonLocationCommand(CannonLocation.coneHighLocation)
      ,
      new WaitCommand(2)
      ,
      new CannonClawOutCommand(cannonSubsystem, -0.8).withTimeout(0.5)
      ,
      new InstantCommand(() -> driveSubsystem.setWheelsToStrafe(90))
      ,
      new SetCannonLocationCommand(CannonLocation.parkLocation)
      ,
      new WaitCommand(.5)
      ,
      new DriveToCoordinateCommand(FieldLocation.humanHalfway, 0.5, 0.2, 180, driveSubsystem)
      ,
      new SetCannonLocationCommand(CannonLocation.lowLocation)
      ,
      new DriveToCoordinateCommand(FieldLocation.humanMiddle, 0.5, 0.2, 0, driveSubsystem)
      ,
      new SetCannonLocationCommand(CannonLocation.sidewaysConeLocation)
      ,
      new CannonClawInCommand(cannonSubsystem, 0.6)
      ,
      new AutoDriveCommand(4.6*12, -20, 0.2, -20, driveSubsystem)
      ,
      new InstantCommand(() -> visionSubsystem.setFrontCameraMode(FrontCameraMode.APRILTAGS))
      ,
      // should we do this or go to the position for leveling?
      new SetCannonLocationCommand(CannonLocation.parkLocation)
      ,
      new DriveToCoordinateCommand(FieldLocation.midMiddle, 0.4, 0.25, 0, driveSubsystem)
      ,
      new BackwardsAutoLevelingCommand(driveSubsystem, cannonSubsystem)
    );
  }
}
