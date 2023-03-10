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

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MidOnlyBalanceAuto extends SequentialCommandGroup {
  PoseOnField otherSide = PoseOnField.fromRedAlliancePositionInMeters(10.9, 3.3);
  /** Creates a new Mid1BalanceAuto. */
  public MidOnlyBalanceAuto(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, CannonSubsystem cannonSubsystem, OdometrySubsystem odometrySubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetInitialNavXOffsetCommand(RobotContainer.navigationSubsystem, driveSubsystem, 180)
      ,
      // tell odometry where we is
      new ZapOdometryCommand(FieldLocation.midStart)
      ,
      new SetNavX180Command()
      ,
      new ZapOdometryCommand(FieldLocation.midStart)
      ,
      new SetCannonLocationCommand(CannonLocation.coneHighLocation)
      ,
      new WaitCommand(2)
      ,
      new CannonClawOutCommand(cannonSubsystem, -0.8).withTimeout(.5)
      ,
      new SetCannonLocationCommand(CannonLocation.parkLocation)
      ,
      new WaitCommand(1)
      ,
      new BackwardsAutoLevelCommunityCommand(driveSubsystem, cannonSubsystem)
    );
  }
}
