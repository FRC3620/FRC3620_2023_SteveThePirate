// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.sql.Time;

import org.usfirst.frc3620.misc.PoseOnField;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
 * Start/place at wall, go out and grab piece from behind via odometry, place piece, balance
 */
public class Wall2BackwardsGrabBalanceAuto extends SequentialCommandGroup {
  /** Creates a new Human2BackwardsGrabNoBalanceAuto. */
  public Wall2BackwardsGrabBalanceAuto(DriveSubsystem driveSubsystem, CannonSubsystem cannonSubsystem, VisionSubsystem visionSubsystem, OdometrySubsystem odometrySubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetInitialNavXOffsetCommand(RobotContainer.navigationSubsystem, driveSubsystem, 180)
      ,
      new InstantCommand(() -> driveSubsystem.fixRelativeEncoders())
      ,
      new InstantCommand(() -> driveSubsystem.setWheelsToStrafe(90))
      ,
      new ZapOdometryCommand(FieldLocation.wallStart)
      ,
      new WaitForValidExtensionEncoderCommand()
      ,
      new SetCannonLocationCommand(CannonLocation.coneHighLocation)
      ,
      new ParallelDeadlineGroup(
        new WaitCommand(0.6)
        ,
        new CannonClawInCommand(cannonSubsystem, 0.5)
      )
      ,
      new WaitCommand(0.4)
      ,
      new CannonClawOutCommand(cannonSubsystem, -1.0).withTimeout(.4) //time is .4
      ,
      new SetCannonLocationCommand(CannonLocation.backwardsHalfwayLocation)
      ,
      new WaitCommand(0.75)
      ,
      new DriveToCoordinateCommand(FieldLocation.wallPickupBehindPre, 0.9, 0.2, 180, driveSubsystem) //was .6 speed
      ,
      new SetCannonLocationCommand(CannonLocation.backwardsFloorPickupLocation)
      //,
      //new WaitCommand(660)
      ,
      new ParallelRaceGroup(
        new DriveToCoordinateCommand(FieldLocation.wallPickupBehindPost, .35, 0.1, 180, driveSubsystem) //was .25 speed
        ,
        new CannonClawInCommand(cannonSubsystem, 0.4)
      )
      ,
      //new WaitCommand(999)
      //,
      new SetCannonLocationCommand(CannonLocation.parkLocation)
      ,
      new DriveToCoordinateCommand(FieldLocation.wallCommunity, 0.9, 0.3, 180, driveSubsystem)
      ,
      new SetCannonLocationCommand(CannonLocation.cubeHighLocation)
      ,
      new DriveToCoordinateCommand(FieldLocation.wallPlaceCube, 0.3, 0.1, 180, driveSubsystem)
      ,
      new WaitCommand(0.25)
      ,
      new CannonClawOutCommand(cannonSubsystem, -0.8).withTimeout(0.4)
      ,
      new SetCannonLocationCommand(CannonLocation.parkLocation)
      ,
      new DriveToCoordinateCommand(FieldLocation.midCommunityWall, 0.8, 0.2, 180, driveSubsystem)
      ,
      new ParallelRaceGroup(
        new BackwardsAutoLevelCommunityCommand(driveSubsystem, cannonSubsystem)
        ,
        new WaitUntilAutoIsDoneCommand(0.4)
      )
      ,
      new XModeCommand(driveSubsystem) 
    );
  }
}
