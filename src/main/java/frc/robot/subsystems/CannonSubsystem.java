package frc.robot.subsystems;

import org.usfirst.frc3620.misc.CANDeviceFinder;
import org.usfirst.frc3620.misc.CANDeviceType;
import org.usfirst.frc3620.misc.CANSparkMaxSendable;
import org.usfirst.frc3620.misc.MotorSetup;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CannonLocation;
import frc.robot.RobotContainer;

public class CannonSubsystem extends SubsystemBase {
  public CannonExtendMechanism cannonExtendMechanism;
  public CannonElevateMechanism cannonElevateMechanism;
  public CannonPitchMechanism cannonPitchMechanism;
  public CannonClawMechanism cannonClawMechanism;
  
  public CANSparkMaxSendable elevation;
  public AnalogInput elevationEncoder;
  
  public CANSparkMaxSendable extend;
  public RelativeEncoder extendEncoder;

  public CANSparkMaxSendable roll;
  public RelativeEncoder rollEncoder;

  public CANSparkMaxSendable pitch;
  public RelativeEncoder pitchEncoder;

  public CANSparkMaxSendable claw;
  public RelativeEncoder clawEncoder;

  /** Creates a new ArmSubsystem. */
  public CannonSubsystem() {
    setupMotors();
    cannonExtendMechanism = new CannonExtendMechanism(extend);
    cannonElevateMechanism = new CannonElevateMechanism(elevation, elevationEncoder);
    cannonPitchMechanism = new CannonPitchMechanism(pitch);
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

  public void setElevation(double height) {
    cannonElevateMechanism.setElevation(height);
  }

  public void disableExtension() {
    cannonExtendMechanism.disable();
  }

  public void setPitch(double pitch) {
    cannonPitchMechanism.setPitch(pitch);
  }

  public void setClawSpeed(double clawSpeed) {
    cannonClawMechanism.setClawSpeed(clawSpeed);
  }

  public double getClawSpeed(){
    return cannonClawMechanism.getClawSpeed();
  }

  public double getRequestedPitch() {
    return cannonPitchMechanism.getRequestedPitch();
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

  void setupMotors() {
    CANDeviceFinder canDeviceFinder = RobotContainer.canDeviceFinder;
		boolean shouldMakeAllCANDevices = RobotContainer.shouldMakeAllCANDevices();

    if (canDeviceFinder.isDevicePresent(CANDeviceType.SPARK_MAX, 9, "Elevation") || shouldMakeAllCANDevices) {
      elevation = new CANSparkMaxSendable(9, MotorType.kBrushless);
      MotorSetup.resetMaxToKnownState(elevation, true);
      elevation.setSmartCurrentLimit(40);
      elevation.setIdleMode(IdleMode.kBrake);
      addChild("elevation", elevation);
    }

		if (canDeviceFinder.isDevicePresent(CANDeviceType.SPARK_MAX, 10, "Extend") || shouldMakeAllCANDevices) {
      extend = new CANSparkMaxSendable(10, MotorType.kBrushless);
      MotorSetup.resetMaxToKnownState(extend, true);
      extend.setSmartCurrentLimit(40);
      extend.setIdleMode(IdleMode.kBrake);
      addChild("extend", extend);
    }

    if(canDeviceFinder.isDevicePresent(CANDeviceType.SPARK_MAX, 11, "Pitch") || shouldMakeAllCANDevices) {
      pitch = new CANSparkMaxSendable(11, MotorType.kBrushless);
      MotorSetup.resetMaxToKnownState(pitch, false);
      pitch.setSmartCurrentLimit(35);
      pitch.setIdleMode(IdleMode.kBrake);
      addChild("pitch", pitch);
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
  }

  public void setLocation(CannonLocation cannonLocation) {
    cannonElevateMechanism.setElevation(cannonLocation.getElevation());
    cannonExtendMechanism.setExtension(cannonLocation.getExtension());
    cannonPitchMechanism.setPitch(cannonLocation.getWristPitch());
  }
}

//:D