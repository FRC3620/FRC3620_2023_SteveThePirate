// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

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
 * Start/place at wall, go out and grab piece from behind via odometry, place piece
 */
public class Wall2BackwardsGrabNoBalanceAuto extends SequentialCommandGroup {
  /** Creates a new Human2BackwardsGrabNoBalanceAuto. */
  public Wall2BackwardsGrabNoBalanceAuto(DriveSubsystem driveSubsystem, CannonSubsystem cannonSubsystem, VisionSubsystem visionSubsystem, OdometrySubsystem odometrySubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    PoseOnField prePickup = FieldLocation.wallPickupBehindPre;
    PoseOnField postPickup = FieldLocation.wallPickupBehindPost;
    if(DriverStation.getAlliance() == Alliance.Blue){
      prePickup = FieldLocation.wallPickupBehindPreBlue;
      postPickup = FieldLocation.wallPickupBehindPostBlue;
    }
    addCommands(
      new SetInitialNavXOffsetCommand(RobotContainer.navigationSubsystem, driveSubsystem, 180)
      ,
      new ZapOdometryCommand(FieldLocation.wallStart)
      ,
      new SetCannonLocationCommand(CannonLocation.halfwayToConeHighLocation)
      ,
      new ParallelDeadlineGroup(
        new WaitCommand(0.6)
        ,
        new CannonClawInCommand(cannonSubsystem, 0.5)
      )
      ,
      new WaitCommand(0.4)
      ,
      new SetCannonLocationCommand(CannonLocation.coneHighLocation)
      ,
      new WaitCommand(1)
      ,
      new CannonClawOutCommand(cannonSubsystem, -0.8).withTimeout(.5)
      ,
      new InstantCommand(() -> driveSubsystem.setWheelsToStrafe(90))
      ,
      new SetCannonLocationCommand(CannonLocation.backwardsHalfwayLocation)
      ,
      new DriveToCoordinateCommand(FieldLocation.wallHalfway, 0.4, 0.1, 180, driveSubsystem)
      ,
      new WaitCommand(0.5)
      ,
      new DriveToCoordinateCommand(prePickup, 0.3, 0.1, 180, driveSubsystem)
      ,
      new SetCannonLocationCommand(CannonLocation.backwardsFloorPickupLocation)
      ,
      new ParallelDeadlineGroup(
        new DriveToCoordinateCommand(postPickup, .2, 0.1, 180, driveSubsystem)
        ,
        new CannonClawInCommand(cannonSubsystem, 0.5)
      )
      ,
      new SetCannonLocationCommand(CannonLocation.parkLocation)
      ,
      new DriveToCoordinateCommand(FieldLocation.wallCommunity, 0.6, 0.2, 180, driveSubsystem)
      ,
      new SetCannonLocationCommand(CannonLocation.cubeHighLocation)
      ,
      new ParallelRaceGroup(
        new DriveToCoordinateCommand(FieldLocation.wallPlaceCube, 0.2, 0.1, 180, driveSubsystem)
        //,
        //new WaitUntilAutoIsDoneCommand(1)
      )
      ,
      new WaitCommand(1)
      ,
      new CannonClawOutCommand(cannonSubsystem, -0.8).withTimeout(0.5)
      ,
      new SetCannonLocationCommand(CannonLocation.parkLocation)
    );
  }
}
