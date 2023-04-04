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
public class Human2BackwardsGrabNoBalanceAuto extends SequentialCommandGroup {
  /** Creates a new Human2BackwardsGrabNoBalanceAuto. */
  public Human2BackwardsGrabNoBalanceAuto(DriveSubsystem driveSubsystem, CannonSubsystem cannonSubsystem, VisionSubsystem visionSubsystem, OdometrySubsystem odometrySubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    double direction = 1;
    double angleLogic = 1;
    PoseOnField cornerOfStation = PoseOnField.fromRedAlliancePositionInMeters(12.068, 4.627);
    if(DriverStation.getAlliance() == Alliance.Blue){
      direction = -1;
      angleLogic = -1;
      cornerOfStation = PoseOnField.fromRedAlliancePositionInMeters(12.068, 4.627 + 0.2);
    }

    addCommands(
      new SetInitialNavXOffsetCommand(RobotContainer.navigationSubsystem, driveSubsystem, 180)
      ,
      new InstantCommand(() -> driveSubsystem.fixRelativeEncoders())
      ,
      new InstantCommand(() -> driveSubsystem.setWheelsToStrafe(90))
      ,
      new ZapOdometryCommand(FieldLocation.humanStart)
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
      new CannonClawOutCommand(cannonSubsystem, -1.0).withTimeout(.3) //time is .4
      ,
      new SetCannonLocationCommand(CannonLocation.backwardsHalfwayLocation)
      ,
      new WaitCommand(0.75)
      ,
      new DriveToCoordinateCommand(FieldLocation.humanPickupBehindPre, 0.9, 0.2, 180, driveSubsystem) //was .6 speed
      ,
      new SetCannonLocationCommand(CannonLocation.backwardsFloorPickupLocation)
      //,
      //new WaitCommand(660)
      ,
      new ParallelRaceGroup(
        //change speed for this definitely
        new DriveToCoordinateCommand(FieldLocation.humanPickupBehindPost, .35, 0.1, 180, driveSubsystem) //was .25 speed
        ,
        new CannonClawInCommand(cannonSubsystem, 0.3)
      )
      ,
      new SetCannonLocationCommand(CannonLocation.parkLocation)
      ,
      new DriveToCoordinateCommand(FieldLocation.humanCommunity, 0.9, 0.3, 180, driveSubsystem)
      ,
      new SetCannonLocationCommand(CannonLocation.cubeHighLocation)
      ,
      new DriveToCoordinateCommand(FieldLocation.humanPlaceCube, 0.4, 0.1, 180, driveSubsystem)
      //,
      //new WaitCommand(0.15)
      ,
      new CannonClawOutCommand(cannonSubsystem, -0.8).withTimeout(0.3)
      ,
      new SetCannonLocationCommand(CannonLocation.backwardsFloorPickupLocation)
      ,
      new DriveToCoordinateCommand(cornerOfStation, 0.9, 0.3, 180, driveSubsystem)
      //,
      //new AutoSpinCommand(-0.5 * direction, angleLogic * 143, driveSubsystem)
      ,
      new ParallelRaceGroup(
        new DriveToCoordinateCommand(FieldLocation.humanGrabSecondPiece, .5, 0.1, 150, driveSubsystem)
        ,
        new CannonClawInCommand(cannonSubsystem, 0.4)
      )
      ,
      new SetCannonLocationCommand(CannonLocation.parkLocation)
      ,
      new DriveToCoordinateCommand(FieldLocation.humanPickupBehindPre, 0.7, 0.15, 180, driveSubsystem)
      ,
      new ParallelRaceGroup(
        new DriveToCoordinateCommand(FieldLocation.humanCommunity, 0.5, 0.15, 180, driveSubsystem)
        ,
        new WaitUntilAutoIsDoneCommand(0.5)
      )
      ,
      new SetCannonLocationCommand(CannonLocation.cubeMidLocation)
      ,
      new DriveToCoordinateCommand(FieldLocation.humanPlaceCube, 0.3, 0.1, 180, driveSubsystem)
      ,
      new CannonClawOutCommand(cannonSubsystem, -0.8).withTimeout(0.4)
    );
  }
}
