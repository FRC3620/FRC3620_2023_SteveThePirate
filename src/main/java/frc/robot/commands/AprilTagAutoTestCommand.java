package frc.robot.commands;

import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.logger.LogCommand;
import org.usfirst.frc3620.logger.EventLogging.Level;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AprilTagAutoTestCommand extends SequentialCommandGroup {
  Logger logger = EventLogging.getLogger(getClass(), Level.INFO);

  public AprilTagAutoTestCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
    addCommands(
      new LocateAprilTagCommand(driveSubsystem, visionSubsystem),
      new StrafeToAprilTagCommand(driveSubsystem, visionSubsystem),
      // strafes from april tag to cone stick 10in
      new AutoDriveCommand(22, 90, 0.3, 0, driveSubsystem),
      new LogCommand(logger, "All done")
    );
  }

}