package frc.robot;

import org.usfirst.frc3620.logger.DataLogger;
import org.usfirst.frc3620.misc.CANDeviceFinder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;

public class RobotDataLogger {
	PowerDistribution powerDistribution = null;
	boolean logDriveMotorCurrent = true;

	public RobotDataLogger (DataLogger dataLogger, CANDeviceFinder canDeviceFinder) {

		dataLogger.addDataProvider("matchTime", () -> DataLogger.f2(DriverStation.getMatchTime()));
		dataLogger.addDataProvider("robotMode", () -> Robot.getCurrentRobotMode().toString());
		dataLogger.addDataProvider("robotModeInt", () -> Robot.getCurrentRobotMode().ordinal());
		dataLogger.addDataProvider("batteryVoltage", () -> DataLogger.f2(RobotController.getBatteryVoltage()));

		dataLogger.addDataProvider("alliance", () -> DriverStation.getAlliance().toString());
		dataLogger.addDataProvider("allianceInt", () -> DriverStation.getAlliance().ordinal());
		dataLogger.addDataProvider("station", () -> DriverStation.getLocation());

		dataLogger.addDataProvider("vision.frontcamera.pipeline_requested", () -> RobotContainer.visionSubsystem.frontCameraMode.getPipelineIndex());
		dataLogger.addDataProvider("vision.frontcamera.pipeline_actual", () -> RobotContainer.visionSubsystem.frontCamera.getPipelineIndex());

		dataLogger.addDataProvider("vision.lastAprilTagTimestamp", () -> RobotContainer.visionSubsystem.getLastAprilTagTimestamp());
		dataLogger.addDataProvider("vision.camera_x", () -> RobotContainer.visionSubsystem.whereIsTheCameraX());
		dataLogger.addDataProvider("vision.camera_y", () -> RobotContainer.visionSubsystem.whereIsTheCameraX());

		dataLogger.addDataProvider("odometry.x", () -> RobotContainer.odometrySubsystem.getPoseMeters().getX());
		dataLogger.addDataProvider("odometry.y", () -> RobotContainer.odometrySubsystem.getPoseMeters().getY());
		
		dataLogger.addDataProvider("nav.heading_raw", () -> DataLogger.f2(RobotContainer.navigationSubsystem.getRawHeading()));
		dataLogger.addDataProvider("nav.heading", () -> DataLogger.f2(RobotContainer.navigationSubsystem.getCorrectedHeading()));
		dataLogger.addDataProvider("nav.heading_offset", () -> DataLogger.f2(RobotContainer.navigationSubsystem.getHeadingOffset()));

		if (canDeviceFinder.isPowerDistributionPresent()) {
			powerDistribution = new PowerDistribution();
			dataLogger.addDataProvider("pdp.totalCurrent", () -> DataLogger.f2(powerDistribution.getTotalCurrent()));
			dataLogger.addDataProvider("pdp.totalPower", () -> DataLogger.f2(powerDistribution.getTotalPower()));
			dataLogger.addDataProvider("pdp.totalEnergy", () -> DataLogger.f2(powerDistribution.getTotalEnergy()));
		}

		if (RobotContainer.cannonSubsystem.elevation != null) {
			dataLogger.addDataProvider("cannon.elevate.motor_current", () -> DataLogger.f2(RobotContainer.cannonSubsystem.elevation.getOutputCurrent()));
			dataLogger.addDataProvider("cannon.elevate.power", () -> DataLogger.f2(RobotContainer.cannonSubsystem.elevation.getAppliedOutput()));
			dataLogger.addDataProvider("cannon.elevate.temperature", () -> DataLogger.f2(RobotContainer.cannonSubsystem.elevation.getMotorTemperature()));
			dataLogger.addDataProvider("cannon.elevate.requested_position", () -> DataLogger.f2(RobotContainer.cannonSubsystem.getRequestedElevation()));
			dataLogger.addDataProvider("cannon.elevate.current_position", () -> DataLogger.f2(RobotContainer.cannonSubsystem.getCurrentElevation()));
		}

		if (RobotContainer.cannonSubsystem.extend != null) {
			dataLogger.addDataProvider("cannon.extension.motor_current", () -> DataLogger.f2(RobotContainer.cannonSubsystem.extend.getOutputCurrent()));
			dataLogger.addDataProvider("cannon.extension.power", () -> DataLogger.f2(RobotContainer.cannonSubsystem.extend.getAppliedOutput()));
			dataLogger.addDataProvider("cannon.extension.temperature", () -> DataLogger.f2(RobotContainer.cannonSubsystem.extend.getMotorTemperature()));
			dataLogger.addDataProvider("cannon.extension.requested_position", () -> DataLogger.f2(RobotContainer.cannonSubsystem.getRequestedExtension()));
			dataLogger.addDataProvider("cannon.extension.current_position", () -> DataLogger.f2(RobotContainer.cannonSubsystem.getCurrentExtension()));
		}

		if (RobotContainer.cannonSubsystem.pitch != null) {
			dataLogger.addDataProvider("cannon.pitch.motor_current", () -> DataLogger.f2(RobotContainer.cannonSubsystem.pitch.getOutputCurrent()));
			dataLogger.addDataProvider("cannon.pitch.power", () -> DataLogger.f2(RobotContainer.cannonSubsystem.pitch.getAppliedOutput()));
			dataLogger.addDataProvider("cannon.pitch.temperature", () -> DataLogger.f2(RobotContainer.cannonSubsystem.pitch.getMotorTemperature()));
			dataLogger.addDataProvider("cannon.pitch.requested_position", () -> DataLogger.f2(RobotContainer.cannonSubsystem.getRequestedPitch()));
			dataLogger.addDataProvider("cannon.pitch.current_position", () -> DataLogger.f2(RobotContainer.cannonSubsystem.getCurrentPitch()));
		}

		if (RobotContainer.cannonSubsystem.claw != null) {
			dataLogger.addDataProvider("cannon.claw.motor_current", () -> DataLogger.f2(RobotContainer.cannonSubsystem.claw.getOutputCurrent()));
			dataLogger.addDataProvider("cannon.claw.power", () -> DataLogger.f2(RobotContainer.cannonSubsystem.claw.getAppliedOutput()));
			dataLogger.addDataProvider("cannon.claw.temperature", () -> DataLogger.f2(RobotContainer.cannonSubsystem.claw.getMotorTemperature()));
		}

		if (RobotContainer.driveSubsystem.leftFrontDrive != null) {
			dataLogger.addDataProvider("drive.lf.azimuth.home_encoder", () -> DataLogger.f2(RobotContainer.driveSubsystem.leftFrontHomeEncoder.getVoltage()));
			dataLogger.addDataProvider("drive.lf.azimuth.encoder", () -> DataLogger.f2(RobotContainer.driveSubsystem.leftFrontAzimuthEncoder.getPosition()));
			dataLogger.addDataProvider("drive.rf.azimuth.home_encoder", () -> DataLogger.f2(RobotContainer.driveSubsystem.rightFrontHomeEncoder.getVoltage()));
			dataLogger.addDataProvider("drive.rf.azimuth.encoder", () -> DataLogger.f2(RobotContainer.driveSubsystem.rightFrontAzimuthEncoder.getPosition()));
			dataLogger.addDataProvider("drive.lb.azimuth.home_encoder", () -> DataLogger.f2(RobotContainer.driveSubsystem.leftBackHomeEncoder.getVoltage()));
			dataLogger.addDataProvider("drive.lb.azimuth.encoder", () -> DataLogger.f2(RobotContainer.driveSubsystem.leftBackAzimuthEncoder.getPosition()));
			dataLogger.addDataProvider("drive.rb.azimuth.home_encoder", () -> DataLogger.f2(RobotContainer.driveSubsystem.rightBackHomeEncoder.getVoltage()));
			dataLogger.addDataProvider("drive.rb.azimuth.encoder", () -> DataLogger.f2(RobotContainer.driveSubsystem.rightBackAzimuthEncoder.getPosition()));

			if (logDriveMotorCurrent) {
				dataLogger.addDataProvider("drive.lf.drive.power", () -> DataLogger.f2(RobotContainer.driveSubsystem.leftFrontDrive.getAppliedOutput()));
				dataLogger.addDataProvider("drive.rf.drive.power", () -> DataLogger.f2(RobotContainer.driveSubsystem.rightFrontDrive.getAppliedOutput()));
				dataLogger.addDataProvider("drive.lb.drive.power", () -> DataLogger.f2(RobotContainer.driveSubsystem.leftBackDrive.getAppliedOutput()));
				dataLogger.addDataProvider("drive.rb.drive.power", () -> DataLogger.f2(RobotContainer.driveSubsystem.rightBackDrive.getAppliedOutput()));
				dataLogger.addDataProvider("drive.lf.drive.motor_current", () -> DataLogger.f2(RobotContainer.driveSubsystem.leftFrontDrive.getOutputCurrent()));
				dataLogger.addDataProvider("drive.rf.drive.motor_current", () -> DataLogger.f2(RobotContainer.driveSubsystem.rightFrontDrive.getOutputCurrent()));
				dataLogger.addDataProvider("drive.lb.drive.motor_current", () -> DataLogger.f2(RobotContainer.driveSubsystem.leftBackDrive.getOutputCurrent()));
				dataLogger.addDataProvider("drive.rb.drive.motor_current", () -> DataLogger.f2(RobotContainer.driveSubsystem.rightBackDrive.getOutputCurrent()));
			}
		}

	}
}