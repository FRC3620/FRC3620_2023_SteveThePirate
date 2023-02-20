// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.usfirst.frc3620.misc.CANDeviceFinder;
import org.usfirst.frc3620.misc.CANDeviceType;
import org.usfirst.frc3620.misc.CANSparkMaxSendable;
import org.usfirst.frc3620.misc.MotorSetup;

import com.revrobotics.AnalogInput;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CannonLocation;
import frc.robot.RobotContainer;


public class CannonSubsystem extends SubsystemBase {
  public CannonExtendMechanism cannonExtendMechanism;
  public CannonElevateMechanism cannonElevateMechanism;
  public CannonPitchMechanism cannonPitchMechanism;
  public CannonClawMechanism cannonClawMechanism;
  
  public CANSparkMaxSendable elevation;
  public RelativeEncoder elevationEncoder;
  public Encoder elevateEncoder;
  public DigitalInput homeSwitch;
  public DigitalInput notHomeSwitch;


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
    cannonElevateMechanism = new CannonElevateMechanism(elevation, elevateEncoder, homeSwitch, notHomeSwitch);
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

  public void setLength(double length) {
    cannonExtendMechanism.setLength(length);
  }

  public void setHeight(double height) {
    cannonElevateMechanism.setHeight(height);
  }


  public void setPitch(double pitch)
  {
    cannonPitchMechanism.setPitch(pitch);
  }

  public void setClawSpeed(double clawSpeed) {
    cannonClawMechanism.setClawSpeed(clawSpeed);
  }

  public double getClawSpeed(){
    return cannonClawMechanism.getClawSpeed();
  }

  public double getElevation() {
    return cannonElevateMechanism.getElevation();
  }
  
  public double getExtension() {
    return cannonExtendMechanism.getExtension();
  }
  
  public double getPitch() {
    return cannonPitchMechanism.getPitch();
  }

  //Sets length of arm for movement. (Endgame?)
  public void setTravel() {
    
  }


  void setupMotors() {
    CANDeviceFinder canDeviceFinder = RobotContainer.canDeviceFinder;
		boolean shouldMakeAllCANDevices = RobotContainer.shouldMakeAllCANDevices();

    if (canDeviceFinder.isDevicePresent(CANDeviceType.SPARK_MAX, 9, "Elevation") || shouldMakeAllCANDevices) {
      elevation = new CANSparkMaxSendable(9, MotorType.kBrushless);
      MotorSetup.resetMaxToKnownState(elevation, true);
      addChild("elevation", elevation);
    }

		if (canDeviceFinder.isDevicePresent(CANDeviceType.SPARK_MAX, 10, "Extend") || shouldMakeAllCANDevices) {
      extend = new CANSparkMaxSendable(10, MotorType.kBrushless);
      MotorSetup.resetMaxToKnownState(extend, true);
      addChild("extend", extend);
    }

    if(canDeviceFinder.isDevicePresent(CANDeviceType.SPARK_MAX, 11, "Pitch") || shouldMakeAllCANDevices) {
      pitch = new CANSparkMaxSendable(11, MotorType.kBrushless);
      MotorSetup.resetMaxToKnownState(pitch, false);
      addChild("pitch", pitch);
    }

    if(canDeviceFinder.isDevicePresent(CANDeviceType.SPARK_MAX, 12, "Claw") || shouldMakeAllCANDevices) {
      claw = new CANSparkMaxSendable(12, MotorType.kBrushless);
      MotorSetup.resetMaxToKnownState(claw, false);
      claw.setSmartCurrentLimit(5);

      addChild("claw", claw);
    }
    elevateEncoder = new Encoder(1, 2);
    addChild("elevateEncoder", elevateEncoder);

    homeSwitch = new DigitalInput(3);
    addChild("homeSwitch", homeSwitch);

    notHomeSwitch = new DigitalInput(4);
    addChild("not home switch", notHomeSwitch);

  }

  public void setLocation(CannonLocation cannonLocation) {
    cannonElevateMechanism.setHeight(cannonLocation.getElevation());
    cannonExtendMechanism.setLength(cannonLocation.getExtension());
    cannonPitchMechanism.setPitch(cannonLocation.getWristPitch());
  }
}

































































//:D