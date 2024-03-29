// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.usfirst.frc3620.misc.PoseOnField;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
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
 * Start/place piece at mid, go over charge station and grab piece, balance
 */
public class Mid1NoGrabBalanceAuto extends SequentialCommandGroup {

  /** Creates a new Mid1BalanceAuto. */
  public Mid1NoGrabBalanceAuto(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem,
      CannonSubsystem cannonSubsystem, OdometrySubsystem odometrySubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetInitialNavXOffsetCommand(RobotContainer.navigationSubsystem, driveSubsystem, 180)
      ,
      // tell odometry where we is
      new ZapOdometryCommand(FieldLocation.midStart)
      ,
      new WaitForValidExtensionEncoderCommand()
      ,
      new SetCannonLocationCommand(CannonLocation.coneHighLocation)
      ,
      new ParallelDeadlineGroup(
        new WaitCommand(0.6)
        ,
        new CannonClawInCommand(cannonSubsystem, 0.8)
      )
      ,
      new WaitCommand(0.4)
      ,
      new CannonClawOutCommand(cannonSubsystem, -0.8).withTimeout(.5)
      ,
      // cone got spit out
      new SetCannonLocationCommand(CannonLocation.parkLocation)
      ,
      new InstantCommand(() -> driveSubsystem.setWheelsToStrafe(90))
      ,
      new WaitCommand(.5)
      ,
      new DriveToCoordinateCommand(FieldLocation.midPickupBehindPre, 0.4, 0.1, 180, driveSubsystem)
      ,
      new WaitCommand(0.25)
      ,
      new DriveToCoordinateCommand(FieldLocation.midMiddle, 0.4, 0.2, 180, driveSubsystem)
      ,
      new AutoLevelNoCounterCommand(driveSubsystem, cannonSubsystem)
    );
  }
}
