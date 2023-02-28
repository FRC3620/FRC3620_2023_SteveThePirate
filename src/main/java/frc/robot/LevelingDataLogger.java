package frc.robot;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.OdometrySubsystem;
import org.usfirst.frc3620.logger.FastDataLoggerCollections;
import org.usfirst.frc3620.logger.IFastDataLogger;

import java.util.Date;

public class LevelingDataLogger {
    public static IFastDataLogger getDataLogger (String name, ILevelingDataSource iLevelingDataSource) {
        return getDataLogger(name, iLevelingDataSource, 15.0);
    }

    public static IFastDataLogger getDataLogger (String name, ILevelingDataSource iLevelingDataSource, double length) {
        OdometrySubsystem odometrySubsystem = RobotContainer.odometrySubsystem;

        IFastDataLogger dataLogger = new FastDataLoggerCollections();
        dataLogger.setInterval(0.0);
        dataLogger.setMaxLength(length);
        dataLogger.setFilename(name);
        Date timestamp = new Date();
        dataLogger.setFilenameTimestamp(timestamp);

        dataLogger.addMetadata("timestamp", timestamp.toString());

        dataLogger.addDataProvider("state", () -> iLevelingDataSource.getLevelingData().levelingState);
        dataLogger.addDataProvider("stateInt", () -> iLevelingDataSource.getLevelingData().levelingState.ordinal());
        dataLogger.addDataProvider("pitch", () -> iLevelingDataSource.getLevelingData().pitch);

        dataLogger.addDataProvider("navx.heading", () -> RobotContainer.navigationSubsystem.getCorrectedHeading());

        dataLogger.addDataProvider("pitch", () -> iLevelingDataSource.getLevelingData().getPitch());
        dataLogger.addDataProvider("state", () -> iLevelingDataSource.getLevelingData().getLevelingState());

        dataLogger.addDataProvider("odometry.x", () -> odometrySubsystem.getPoseMeters().getX());
        dataLogger.addDataProvider("odometry.y", () -> odometrySubsystem.getPoseMeters().getY());

        dataLogger.addDataProvider("lf.drive.velocity", () -> RobotContainer.driveSubsystem.getCornerDriveVelocity(DriveSubsystem.Corner.LF));
        dataLogger.addDataProvider("rf.drive.velocity", () -> RobotContainer.driveSubsystem.getCornerDriveVelocity(DriveSubsystem.Corner.RF));
        dataLogger.addDataProvider("lb.drive.velocity", () -> RobotContainer.driveSubsystem.getCornerDriveVelocity(DriveSubsystem.Corner.LB));
        dataLogger.addDataProvider("rb.drive.velocity", () -> RobotContainer.driveSubsystem.getCornerDriveVelocity(DriveSubsystem.Corner.RB));

        dataLogger.addDataProvider("lf.drive.position", () -> RobotContainer.driveSubsystem.getCornerDrivePosition(DriveSubsystem.Corner.LF));
        dataLogger.addDataProvider("rf.drive.position", () -> RobotContainer.driveSubsystem.getCornerDrivePosition(DriveSubsystem.Corner.RF));
        dataLogger.addDataProvider("lb.drive.position", () -> RobotContainer.driveSubsystem.getCornerDrivePosition(DriveSubsystem.Corner.LB));
        dataLogger.addDataProvider("rb.drive.position", () -> RobotContainer.driveSubsystem.getCornerDrivePosition(DriveSubsystem.Corner.RB));

        dataLogger.addDataProvider("lf.azimuth.position", () -> RobotContainer.driveSubsystem.getCornerAzimuthPosition(DriveSubsystem.Corner.LF));
        dataLogger.addDataProvider("rf.azimuth.position", () -> RobotContainer.driveSubsystem.getCornerAzimuthPosition(DriveSubsystem.Corner.RF));
        dataLogger.addDataProvider("lb.azimuth.position", () -> RobotContainer.driveSubsystem.getCornerAzimuthPosition(DriveSubsystem.Corner.LB));
        dataLogger.addDataProvider("rb.azimuth.position", () -> RobotContainer.driveSubsystem.getCornerAzimuthPosition(DriveSubsystem.Corner.RB));

        return dataLogger;
    }
}