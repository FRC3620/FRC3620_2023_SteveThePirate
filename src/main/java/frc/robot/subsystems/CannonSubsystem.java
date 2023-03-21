package frc.robot.subsystems;

import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.logger.EventLogging.Level;
import org.usfirst.frc3620.misc.CANDeviceFinder;
import org.usfirst.frc3620.misc.CANDeviceType;
import org.usfirst.frc3620.misc.CANSparkMaxSendable;
import org.usfirst.frc3620.misc.MotorSetup;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CannonLocation;
import frc.robot.RobotContainer;

public class CannonSubsystem extends SubsystemBase {
  Logger logger = EventLogging.getLogger(getClass(), Level.INFO);
  public CannonExtendMechanism cannonExtendMechanism;
  public CannonElevateMechanism cannonElevateMechanism;
  public CannonPitchMechanism cannonPitchMechanism;
  public CannonClawMechanism cannonClawMechanism;
  
  public CANSparkMaxSendable elevation;
  public RelativeEncoder elevationMotorEncoder;
  public AnalogInput elevationEncoder;

  public CANSparkMaxSendable elevation2;
  
  public CANSparkMaxSendable extend;
  public RelativeEncoder extendEncoder;

  public CANSparkMaxSendable extend2;

  public CANSparkMaxSendable roll;
  public RelativeEncoder rollEncoder;

  public CANSparkMaxSendable pitch;
  public RelativeEncoder pitchMotorEncoder;
  public Encoder pitchEncoder;

  public CANSparkMaxSendable claw;
  public RelativeEncoder clawEncoder;

  /** Creates a new ArmSubsystem. */
  public CannonSubsystem() {
    setupMotors();
    cannonExtendMechanism = new CannonExtendMechanism(extend, extend2);
    cannonElevateMechanism = new CannonElevateMechanism(elevation, elevationEncoder, elevationMotorEncoder, elevation2);
    cannonPitchMechanism = new CannonPitchMechanism(pitch, pitchEncoder, pitchMotorEncoder);
    cannonClawMechanism = new CannonClawMechanism(claw);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    cannonExtendMechanism.periodic();
    cannonElevateMechanism.periodic();
    cannonPitchMechanism.periodic();
    cannonClawMechanism.periodic();
  }

  public void setExtension(double length) {
    cannonExtendMechanism.setExtension(length);
  }

  public void extendCannon(double power) {
    cannonExtendMechanism.extendCannon(power);
  }

  public void disableExtension() {
    cannonExtendMechanism.disable();
  }

  public void setElevation(double height) {
    cannonElevateMechanism.setElevation(height);
  }

  public void elevateCannon(double power) {
    cannonElevateMechanism.elevateCannon(power);
  }

  public void setPitch(double pitch) {
    cannonPitchMechanism.setPitch(pitch);
  }

  public void pitchCannon(double power) {
    cannonPitchMechanism.pitchCannon(power);
  }

  public void setClawPower(double clawSpeed) {
    cannonClawMechanism.setClawPower(clawSpeed);
  }

  public double getClawPower(){
    return cannonClawMechanism.getClawPower();
  }

  public double getRequestedPitch() {
    return cannonPitchMechanism.getRequestedPitch();
  }

  public double getClampedPitch() {
    return cannonPitchMechanism.getClampedPitch();
  }

  public double getCurrentPitch() {
    return cannonPitchMechanism.getCurrentPitch();
  }

  public double getRequestedElevation() {
    return cannonElevateMechanism.getRequestedElevation();
  }
  
  public double getCurrentElevation() {
    return cannonElevateMechanism.getCurrentElevation();
  }

  public double getRequestedExtension() {
    return cannonExtendMechanism.getRequestedExtension();
  }

  public double getCurrentExtension() {
    return cannonExtendMechanism.getCurrentExtension();
  }

  public double getAdjustedRequestedExtension() {
    return cannonExtendMechanism.getAdjustedRequestedExtension();
  }

  public void recalibrataePitch() {
    cannonPitchMechanism.recalibrataePitch();
  }

  void setupMotors() {
    CANDeviceFinder canDeviceFinder = RobotContainer.canDeviceFinder;
		boolean shouldMakeAllCANDevices = RobotContainer.shouldMakeAllCANDevices();

    if (canDeviceFinder.isDevicePresent(CANDeviceType.SPARK_MAX, 9, "Elevation") || shouldMakeAllCANDevices) {
      elevation = new CANSparkMaxSendable(9, MotorType.kBrushless);
      MotorSetup.resetMaxToKnownState(elevation, true);
      elevation.setSmartCurrentLimit(20);
      elevation.setIdleMode(IdleMode.kBrake);
      addChild("elevation", elevation);

      elevationMotorEncoder = elevation.getEncoder();

      // motors are geared 5:1, 5:1, bull gear is 50/30, chain drive is 75:15.
      double ratio = 360.0 / (5.0 * 5.0 * (50.0 / 30.0) * (75.0 / 15.0));
      elevationMotorEncoder.setPositionConversionFactor(ratio);
      elevationMotorEncoder.setVelocityConversionFactor(ratio);

      logger.info ("Elevation motor position scale = {}", elevationMotorEncoder.getPositionConversionFactor());
    }

    if (canDeviceFinder.isDevicePresent(CANDeviceType.SPARK_MAX, 14, "Elevate2") || shouldMakeAllCANDevices) {
      elevation2 = new CANSparkMaxSendable(14, MotorType.kBrushless);
      // the inverted is ignored if this is a follower
      MotorSetup.resetMaxToKnownState(elevation2, true);
      elevation2.setSmartCurrentLimit(20);
      elevation2.setIdleMode(IdleMode.kBrake);
      addChild("elevate2", elevation2);
    }

		if (canDeviceFinder.isDevicePresent(CANDeviceType.SPARK_MAX, 10, "Extend") || shouldMakeAllCANDevices) {
      extend = new CANSparkMaxSendable(10, MotorType.kBrushless);
      MotorSetup.resetMaxToKnownState(extend, true);
      extend.setSmartCurrentLimit(40);
      extend.setIdleMode(IdleMode.kBrake);
      addChild("extend", extend);
    }

    if (canDeviceFinder.isDevicePresent(CANDeviceType.SPARK_MAX, 13, "Extend2") || shouldMakeAllCANDevices) {
      extend2 = new CANSparkMaxSendable(13, MotorType.kBrushless);
      // the inverted is ignored if this is a follower
      MotorSetup.resetMaxToKnownState(extend2, true);
      extend2.setSmartCurrentLimit(40);
      extend2.setIdleMode(IdleMode.kBrake);
      addChild("extend2", extend2);
    }

    if(canDeviceFinder.isDevicePresent(CANDeviceType.SPARK_MAX, 11, "Pitch") || shouldMakeAllCANDevices) {
      pitch = new CANSparkMaxSendable(11, MotorType.kBrushless);
      MotorSetup.resetMaxToKnownState(pitch, false);
      pitch.setSmartCurrentLimit(40);
      pitch.setIdleMode(IdleMode.kBrake);
      addChild("pitch", pitch);

      pitchMotorEncoder = pitch.getEncoder();
      // 360 to convert from rotations to degrees
      // denominator is a 4:1 gearbox and a 1/18:42 pulley reduction
      double ratio = 360.0 / (4.0 * (42.0 / 18.0));
      pitchMotorEncoder.setPositionConversionFactor(ratio);
      pitchMotorEncoder.setVelocityConversionFactor(ratio);
      logger.info("Pitch motor position scale = {}", pitchMotorEncoder.getPositionConversionFactor());
    }

    if(canDeviceFinder.isDevicePresent(CANDeviceType.SPARK_MAX, 12, "Claw") || shouldMakeAllCANDevices) {
      claw = new CANSparkMaxSendable(12, MotorType.kBrushless);
      MotorSetup.resetMaxToKnownState(claw, false);
      claw.setSmartCurrentLimit(20);
      claw.setIdleMode(IdleMode.kBrake);

      addChild("claw", claw);
    }

    elevationEncoder = new AnalogInput(4);
    addChild("elevationEncoder", elevationEncoder);

    pitchEncoder = new Encoder(5,6);
    addChild("pitchEncoder", pitchEncoder);
  }

  public void setLocation(CannonLocation cannonLocation) {
    cannonElevateMechanism.setElevation(cannonLocation.getElevation());
    cannonExtendMechanism.setExtension(cannonLocation.getExtension());
    cannonPitchMechanism.setPitch(cannonLocation.getWristPitch());
  }
}

//:D