package frc.robot.commands;

import java.text.DecimalFormat;

import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.logger.EventLogging.Level;
import org.usfirst.frc3620.misc.PoseOnField;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.OdometrySubsystem;

public class ZapOdometryCommand extends InstantCommand {
  Logger logger = EventLogging.getLogger(getClass(), Level.INFO);

  OdometrySubsystem odometrySubsystem;
  PoseOnField whereIAm;

  public ZapOdometryCommand(PoseOnField whereIAm) {
    this.whereIAm = whereIAm;
    odometrySubsystem = RobotContainer.odometrySubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Alliance alliance = DriverStation.getAlliance();
    Pose2d was = odometrySubsystem.getPoseMeters();
    Translation2d want = whereIAm.getTranslationInMeters(alliance);
    odometrySubsystem.resetPosition(alliance, want);
    logger.info ("was at {}, {}; want to set to {}, {} (color {})",
      f2(was.getX()), f2(was.getY()), f2(want.getX()), f2(want.getY()), alliance.toString());

    Translation2d isNow = odometrySubsystem.getPoseMeters().getTranslation();
    double error = isNow.getDistance(want);
    if (error > 0.1) {
      logger.error ("didn't stick, we are now at {}, {}", f2(isNow.getX()), f2(isNow.getY()));
    }
  }

  DecimalFormat f2format = new DecimalFormat("#.##");

  String f2 (double d) {
    return f2format.format(d);
  }
}
