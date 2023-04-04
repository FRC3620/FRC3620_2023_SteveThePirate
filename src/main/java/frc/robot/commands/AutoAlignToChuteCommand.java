// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.usfirst.frc3620.misc.PoseOnField;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.CannonLocation;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoAlignToChuteCommand extends SequentialCommandGroup {
  /** Creates a new AutoAlignToChuteCommand. */
  public AutoAlignToChuteCommand(DriveSubsystem driveSubsystem) {
    PoseOnField coord = PoseOnField.fromRedAlliancePositionInMeters(2.5, 7);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DriveToCoordinateCommand(coord, 0.2, 0.1, 0, driveSubsystem)
      ,
      new AutoSpinCommand(0.2, 90, driveSubsystem)
      ,
      new SetCannonLocationCommand(CannonLocation.chuteLocation)
    );
  }
}
