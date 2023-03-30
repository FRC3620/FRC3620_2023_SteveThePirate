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
 * Start/place at human, go out and grab piece from behind via odometry, place piece
 */
public class Human2BackwardsGrabBalanceAuto extends SequentialCommandGroup {
  /** Creates a new Human2BackwardsGrabNoBalanceAuto. */
  public Human2BackwardsGrabBalanceAuto(DriveSubsystem driveSubsystem, CannonSubsystem cannonSubsystem, VisionSubsystem visionSubsystem, OdometrySubsystem odometrySubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    double direction = 1;
    double angleLogic = 1;
    PoseOnField prePickup = FieldLocation.humanPickupBehindPre;
    PoseOnField postPickup = FieldLocation.humanPickupBehindPost;
    if(DriverStation.getAlliance() == Alliance.Blue){
      direction = -1;
      angleLogic = -1;
      prePickup = FieldLocation.humanPickupBehindPreBlue;
      postPickup = FieldLocation.humanPickupBehindPostBlue;
    }

    addCommands(
      new SetInitialNavXOffsetCommand(RobotContainer.navigationSubsystem, driveSubsystem, 180)
      ,
      new InstantCommand(() -> driveSubsystem.setWheelsToStrafe(90))
      ,
      new ZapOdometryCommand(FieldLocation.humanStart)
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
      //,
      //new WaitCommand(1)
      ,
      new CannonClawOutCommand(cannonSubsystem, -1.0).withTimeout(.4) //time is .4
      ,
      new SetCannonLocationCommand(CannonLocation.backwardsHalfwayLocation)
      ,
      new DriveToCoordinateCommand(prePickup, 0.35, 0.2, 180, driveSubsystem) //was .6 speed
      ,
      new SetCannonLocationCommand(CannonLocation.backwardsFloorPickupLocation)
      ,
      new WaitCommand(0)



      ,
      new ParallelRaceGroup(
        new DriveToCoordinateCommand(postPickup, .25, 0.1, 180, driveSubsystem) //was .25 speed
        ,
        new CannonClawInCommand(cannonSubsystem, 0.4)
      )
      ,
      new SetCannonLocationCommand(CannonLocation.parkLocation)
      ,
      new DriveToCoordinateCommand(FieldLocation.humanCommunity, 0.8, 0.3, 180, driveSubsystem)
      ,
      new SetCannonLocationCommand(CannonLocation.cubeHighLocation)
      ,
      new DriveToCoordinateCommand(FieldLocation.humanPlaceCube, 0.3, 0.1, 180, driveSubsystem)
      ,
      new WaitCommand(0.5)
      ,
      new CannonClawOutCommand(cannonSubsystem, -0.8).withTimeout(0.5)
      ,
      new SetCannonLocationCommand(CannonLocation.parkLocation)
      ,
      new DriveToCoordinateCommand(FieldLocation.midCommunity, 0.5, 0.2, 180, driveSubsystem)
      ,
      new BackwardsAutoLevelCommunityCommand(driveSubsystem, cannonSubsystem)
    );
  }
}
