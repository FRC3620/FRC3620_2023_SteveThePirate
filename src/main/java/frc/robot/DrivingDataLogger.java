package frc.robot;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.OdometrySubsystem;
import org.usfirst.frc3620.logger.FastDataLoggerCollections;
import org.usfirst.frc3620.logger.IFastDataLogger;

import java.util.Date;

public class DrivingDataLogger {
    public static IFastDataLogger getShootingDataLogger (String name) {
        return getShootingDataLogger(name, 15.0);
    }

    public static IFastDataLogger getShootingDataLogger (String name, double length) {
        OdometrySubsystem odometrySubsystem = RobotContainer.odometrySubsystem;

        IFastDataLogger dataLogger = new FastDataLoggerCollections();
        dataLogger.setInterval(0.0);
        dataLogger.setMaxLength(length);
        dataLogger.setFilename(name);
        Date timestamp = new Date();
        dataLogger.setFilenameTimestamp(timestamp);

        dataLogger.addMetadata("timestamp", timestamp.toString());

        dataLogger.addDataProvider("navx.heading", () -> RobotContainer.navigationSubsystem.getCorrectedHeading());

        dataLogger.addDataProvider("odometry.x", () -> odometrySubsystem.getPoseMeters().getX());
        dataLogger.addDataProvider("odometry.y", () -> odometrySubsystem.getPoseMeters().getY());

        dataLogger.addDataProvider("lf.drive.velocity", () -> RobotContainer.driveSubsystem.getCornerDriveVelocity(DriveSubsystem.Corner.LF));
        dataLogger.addDataProvider("rf.drive.velocity", () -> RobotContainer.driveSubsystem.getCornerDriveVelocity(DriveSubsystem.Corner.RF));
        dataLogger.addDataProvider("lb.drive.velocity", () -> RobotContainer.driveSubsystem.getCornerDriveVelocity(DriveSubsystem.Corner.LB));
        dataLogger.addDataProvider("rb.drive.velocity", () -> RobotContainer.driveSubsystem.getCornerDriveVelocity(DriveSubsystem.Corner.RB));

        dataLogger.addDataProvider("lf.azimuth.position", () -> RobotContainer.driveSubsystem.getCornerAzimuthPosition(DriveSubsystem.Corner.LF));
        dataLogger.addDataProvider("rf.azimuth.position", () -> RobotContainer.driveSubsystem.getCornerAzimuthPosition(DriveSubsystem.Corner.RF));
        dataLogger.addDataProvider("lb.azimuth.position", () -> RobotContainer.driveSubsystem.getCornerAzimuthPosition(DriveSubsystem.Corner.LB));
        dataLogger.addDataProvider("rb.azimuth.position", () -> RobotContainer.driveSubsystem.getCornerAzimuthPosition(DriveSubsystem.Corner.RB));

        return dataLogger;
    }
}