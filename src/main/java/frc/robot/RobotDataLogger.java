package frc.robot;

import org.usfirst.frc3620.logger.DataLogger;
import org.usfirst.frc3620.misc.CANDeviceFinder;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.subsystems.CannonSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystem.Corner;

public class RobotDataLogger {
	PowerDistribution powerDistribution = null;
	boolean logDriveMotorCurrent = true;
	DriveSubsystem driveSubsystem = RobotContainer.driveSubsystem;
	CannonSubsystem cannonSubsystem = RobotContainer.cannonSubsystem;

	public RobotDataLogger (DataLogger dataLogger, CANDeviceFinder canDeviceFinder) {
		BuiltInAccelerometer accelerometer = new BuiltInAccelerometer();

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
		dataLogger.addDataProvider("vision.camera_x", () -> RobotContainer.visionSubsystem.whereIsTheCenterOfTheRobotX());
		dataLogger.addDataProvider("vision.camera_y", () -> RobotContainer.visionSubsystem.whereIsTheCenterOfTheRobotX());

		dataLogger.addDataProvider("odometry.x", () -> RobotContainer.odometrySubsystem.getPoseMeters().getX());
		dataLogger.addDataProvider("odometry.y", () -> RobotContainer.odometrySubsystem.getPoseMeters().getY());
		
		dataLogger.addDataProvider("nav.heading_raw", () -> DataLogger.f2(RobotContainer.navigationSubsystem.getRawHeading()));
		dataLogger.addDataProvider("nav.heading", () -> DataLogger.f2(RobotContainer.navigationSubsystem.getCorrectedHeading()));
		dataLogger.addDataProvider("nav.heading_offset", () -> DataLogger.f2(RobotContainer.navigationSubsystem.getHeadingOffset()));
		dataLogger.addDataProvider("nav.pitch", () -> DataLogger.f2(RobotContainer.navigationSubsystem.getPitch()));
		dataLogger.addDataProvider("nav.roll", () -> DataLogger.f2(RobotContainer.navigationSubsystem.getRoll()));

		dataLogger.addDataProvider("drive.requested_heading", () -> DataLogger.f2(RobotContainer.driveSubsystem.getTargetHeading()));
		dataLogger.addDataProvider("drive.spin_power", () -> DataLogger.f2(RobotContainer.driveSubsystem.getSpinPower()));
		dataLogger.addDataProvider("drive.manual_spin_mode", () -> RobotContainer.driveSubsystem.getForcedManualMode() ? 1 : 0);

		dataLogger.addDataProvider("accel.x", () -> accelerometer.getX());
		dataLogger.addDataProvider("accel.y", () -> accelerometer.getY());
		dataLogger.addDataProvider("accel.z", () -> accelerometer.getZ());

		if (canDeviceFinder.isPowerDistributionPresent()) {
			powerDistribution = new PowerDistribution();
			dataLogger.addDataProvider("pdp.totalCurrent", () -> DataLogger.f2(powerDistribution.getTotalCurrent()));
			dataLogger.addDataProvider("pdp.totalPower", () -> DataLogger.f2(powerDistribution.getTotalPower()));
			dataLogger.addDataProvider("pdp.totalEnergy", () -> DataLogger.f2(powerDistribution.getTotalEnergy()));
		}

		if (cannonSubsystem.elevation != null) {
			dataLogger.addDataProvider("cannon.elevate.motor_current", () -> DataLogger.f2(cannonSubsystem.elevation.getOutputCurrent()));
			dataLogger.addDataProvider("cannon.elevate.power", () -> DataLogger.f2(cannonSubsystem.elevation.getAppliedOutput()));
			dataLogger.addDataProvider("cannon.elevate.temperature", () -> DataLogger.f2(cannonSubsystem.elevation.getMotorTemperature()));
			dataLogger.addDataProvider("cannon.elevate.requested_position", () -> DataLogger.f2(cannonSubsystem.getRequestedElevation()));
			dataLogger.addDataProvider("cannon.elevate.current_position", () -> DataLogger.f2(cannonSubsystem.getCurrentElevation()));
			dataLogger.addDataProvider("cannon.elevate.current_motor_position", () -> DataLogger.f2(cannonSubsystem.elevationMotorEncoder.getPosition()));
			dataLogger.addDataProvider("cannon.elevate.current_motor_velocity", () -> DataLogger.f2(cannonSubsystem.elevationMotorEncoder.getVelocity()));
		}

		if (cannonSubsystem.elevation2 != null) {
			dataLogger.addDataProvider("cannon.elevate2.motor_current",
					() -> DataLogger.f2(cannonSubsystem.elevation2.getOutputCurrent()));
			dataLogger.addDataProvider("cannon.elevate2.power",
					() -> DataLogger.f2(cannonSubsystem.elevation2.getAppliedOutput()));
			dataLogger.addDataProvider("cannon.elevate2.temperature",
					() -> DataLogger.f2(cannonSubsystem.elevation2.getMotorTemperature()));
		}

		if (cannonSubsystem.extend != null) {
			dataLogger.addDataProvider("cannon.extension.motor_current", () -> DataLogger.f2(cannonSubsystem.extend.getOutputCurrent()));
			dataLogger.addDataProvider("cannon.extension.power", () -> DataLogger.f2(cannonSubsystem.extend.getAppliedOutput()));
			dataLogger.addDataProvider("cannon.extension.temperature", () -> DataLogger.f2(cannonSubsystem.extend.getMotorTemperature()));
			dataLogger.addDataProvider("cannon.extension.requested_position", () -> DataLogger.f2(cannonSubsystem.getRequestedExtension()));
			dataLogger.addDataProvider("cannon.extension.current_position", () -> DataLogger.f2(cannonSubsystem.getCurrentExtension()));
		}

		if (cannonSubsystem.extend2 != null) {
			dataLogger.addDataProvider("cannon.extension2.motor_current",
					() -> DataLogger.f2(cannonSubsystem.extend2.getOutputCurrent()));
			dataLogger.addDataProvider("cannon.extension2.power",
					() -> DataLogger.f2(cannonSubsystem.extend2.getAppliedOutput()));
			dataLogger.addDataProvider("cannon.extension2.temperature",
					() -> DataLogger.f2(cannonSubsystem.extend2.getMotorTemperature()));
		}

		if (cannonSubsystem.pitch != null) {
			dataLogger.addDataProvider("cannon.pitch.motor_current", () -> DataLogger.f2(cannonSubsystem.pitch.getOutputCurrent()));
			dataLogger.addDataProvider("cannon.pitch.power", () -> DataLogger.f2(cannonSubsystem.pitch.getAppliedOutput()));
			dataLogger.addDataProvider("cannon.pitch.temperature", () -> DataLogger.f2(cannonSubsystem.pitch.getMotorTemperature()));
			dataLogger.addDataProvider("cannon.pitch.requested_position", () -> cannonSubsystem.getRequestedPitch());
			dataLogger.addDataProvider("cannon.pitch.clamped_requested_position", () -> cannonSubsystem.getClampedPitch());
			dataLogger.addDataProvider("cannon.pitch.current_grayhill_position", () -> DataLogger.f2(cannonSubsystem.getCurrentPitchFromGrayhill()));
			dataLogger.addDataProvider("cannon.pitch.current_motor_position", () -> DataLogger.f2(cannonSubsystem.pitchMotorEncoder.getPosition()));
			dataLogger.addDataProvider("cannon.pitch.current_motor_velocity", () -> DataLogger.f2(cannonSubsystem.pitchMotorEncoder.getVelocity()));
		}

		if (cannonSubsystem.claw != null) {
			dataLogger.addDataProvider("cannon.claw.motor_current", () -> DataLogger.f2(cannonSubsystem.claw.getOutputCurrent()));
			dataLogger.addDataProvider("cannon.claw.power", () -> DataLogger.f2(cannonSubsystem.claw.getAppliedOutput()));
			dataLogger.addDataProvider("cannon.claw.temperature", () -> DataLogger.f2(cannonSubsystem.claw.getMotorTemperature()));
		}

		if (driveSubsystem.leftFrontDrive != null) {
			dataLogger.addDataProvider("drive.lf.azimuth.home_encoder", () -> DataLogger.f2(driveSubsystem.leftFrontHomeEncoder.getVoltage()));
			dataLogger.addDataProvider("drive.lf.azimuth.home_encoder_angle", () -> DataLogger.f2(driveSubsystem.getHomeEncoderHeading(driveSubsystem.leftFrontHomeEncoder)));
			dataLogger.addDataProvider("drive.lf.azimuth.encoder", () -> DataLogger.f2(driveSubsystem.leftFrontAzimuthEncoder.getPosition()));
			dataLogger.addDataProvider("drive.lf.azimuth.encoder_diff", () -> DataLogger.f2(driveSubsystem.encoderDifference(Corner.LF)));
			dataLogger.addDataProvider("drive.rf.azimuth.home_encoder", () -> DataLogger.f2(driveSubsystem.rightFrontHomeEncoder.getVoltage()));
			dataLogger.addDataProvider("drive.rf.azimuth.home_encoder_angle", () -> DataLogger.f2(driveSubsystem.getHomeEncoderHeading(driveSubsystem.rightFrontHomeEncoder)));
			dataLogger.addDataProvider("drive.rf.azimuth.encoder", () -> DataLogger.f2(driveSubsystem.rightFrontAzimuthEncoder.getPosition()));
			dataLogger.addDataProvider("drive.rf.azimuth.encoder_diff", () -> DataLogger.f2(driveSubsystem.encoderDifference(Corner.RF)));
			dataLogger.addDataProvider("drive.lb.azimuth.home_encoder", () -> DataLogger.f2(driveSubsystem.leftBackHomeEncoder.getVoltage()));
			dataLogger.addDataProvider("drive.lb.azimuth.home_encoder_angle", () -> DataLogger.f2(driveSubsystem.getHomeEncoderHeading(driveSubsystem.leftBackHomeEncoder)));
			dataLogger.addDataProvider("drive.lb.azimuth.encoder", () -> DataLogger.f2(driveSubsystem.leftBackAzimuthEncoder.getPosition()));
			dataLogger.addDataProvider("drive.lb.azimuth.encoder_diff", () -> DataLogger.f2(driveSubsystem.encoderDifference(Corner.LB)));
			dataLogger.addDataProvider("drive.rb.azimuth.home_encoder", () -> DataLogger.f2(driveSubsystem.rightBackHomeEncoder.getVoltage()));
			dataLogger.addDataProvider("drive.rb.azimuth.home_encoder_angle", () -> DataLogger.f2(driveSubsystem.getHomeEncoderHeading(driveSubsystem.leftBackHomeEncoder)));
			dataLogger.addDataProvider("drive.rb.azimuth.encoder", () -> DataLogger.f2(driveSubsystem.rightBackAzimuthEncoder.getPosition()));
			dataLogger.addDataProvider("drive.rb.azimuth.encoder_diff", () -> DataLogger.f2(driveSubsystem.encoderDifference(Corner.RB)));

			if (logDriveMotorCurrent) {
				dataLogger.addDataProvider("drive.lf.drive.power", () -> DataLogger.f2(driveSubsystem.leftFrontDrive.getAppliedOutput()));
				dataLogger.addDataProvider("drive.rf.drive.power", () -> DataLogger.f2(driveSubsystem.rightFrontDrive.getAppliedOutput()));
				dataLogger.addDataProvider("drive.lb.drive.power", () -> DataLogger.f2(driveSubsystem.leftBackDrive.getAppliedOutput()));
				dataLogger.addDataProvider("drive.rb.drive.power", () -> DataLogger.f2(driveSubsystem.rightBackDrive.getAppliedOutput()));
				dataLogger.addDataProvider("drive.lf.drive.motor_current", () -> DataLogger.f2(driveSubsystem.leftFrontDrive.getOutputCurrent()));
				dataLogger.addDataProvider("drive.rf.drive.motor_current", () -> DataLogger.f2(driveSubsystem.rightFrontDrive.getOutputCurrent()));
				dataLogger.addDataProvider("drive.lb.drive.motor_current", () -> DataLogger.f2(driveSubsystem.leftBackDrive.getOutputCurrent()));
				dataLogger.addDataProvider("drive.rb.drive.motor_current", () -> DataLogger.f2(driveSubsystem.rightBackDrive.getOutputCurrent()));
			}
		}

		dataLogger.addDataProvider("driver.joy.x", () -> RobotContainer.getDriveHorizontalJoystick());
		dataLogger.addDataProvider("driver.joy.y", () -> RobotContainer.getDriveVerticalJoystick());
		dataLogger.addDataProvider("driver.joy.spin", () -> RobotContainer.getDriveSpinJoystick());
		dataLogger.addDataProvider("driver.joy.mode", () -> RobotContainer.driverJoystick.getCurrentControllerType().toString());

	}
}