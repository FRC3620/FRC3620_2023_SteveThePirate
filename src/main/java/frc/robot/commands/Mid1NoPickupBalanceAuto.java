// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.usfirst.frc3620.misc.PoseOnField;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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

/**
 * Start/place piece at mid, go over charge station, balance
 */
public class Mid1NoPickupBalanceAuto extends SequentialCommandGroup {
  PoseOnField otherSide = PoseOnField.fromRedAlliancePositionInMeters(10.9, 3.3);
  /** Creates a new Mid1BalanceAuto. */
  public Mid1NoPickupBalanceAuto(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, CannonSubsystem cannonSubsystem, OdometrySubsystem odometrySubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetInitialNavXOffsetCommand(RobotContainer.navigationSubsystem, driveSubsystem, 180)
      ,
      // tell odometry where we is
      new ZapOdometryCommand(FieldLocation.midStart)
      ,

      /*new ParallelCommandGroup(
        new DriveToAprilTagCommand(2, Position.HUMAN, driveSubsystem, visionSubsystem, odometrySubsystem)
        ,
        new SetCannonLocationCommand(CannonLocation.coneHighLocation)
      )
      ,*/ // might add this parallelcommandgroup
      new SetNavX180Command()
      ,
      new ZapOdometryCommand(FieldLocation.midStart)
      ,
      new WaitForValidExtensionEncoderCommand()
      ,
      new SetCannonLocationCommand(CannonLocation.coneHighLocation)
      ,
      new WaitCommand(2)
      ,
      new CannonClawOutCommand(cannonSubsystem, -0.8).withTimeout(1.5)
      ,
      /*new ParallelCommandGroup(
        new SetCannonLocationCommand(CannonLocation.parkLocation)
        ,
        new DriveToCoordinateCommand(FieldLocation.otherSide, 0.2, 0.1, 180, driveSubsystem)
      )
      ,*/
      new SetCannonLocationCommand(CannonLocation.parkLocation)
      ,
      new WaitCommand(1)
      ,
      new DriveToCoordinateCommand(FieldLocation.midMiddle, 0.3, 0.1, 180, driveSubsystem)
      ,
      new AutoLevelNoCounterCommand(driveSubsystem, cannonSubsystem)
    );
  }
}
