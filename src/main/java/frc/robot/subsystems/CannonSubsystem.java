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
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;


public class CannonSubsystem extends SubsystemBase {
  public CannonExtendMechanism cannonExtendMechanism;
  public CannonElevateMechanism cannonElevateMechanism;
  public CannonRollMechanism cannonRollMechanism;
  public CannonPitchMechanism cannonPitchMechanism;
  
  public CANSparkMaxSendable elevation;
  public RelativeEncoder elevationEncoder;
  public AnalogInput elevationHomeEncoder;
  public Encoder elevateEncoder;


  public CANSparkMaxSendable extend;
  public RelativeEncoder extendEncoder;
  public AnalogInput extendHomeEncoder;

  public CANSparkMaxSendable roll;
  public RelativeEncoder rollEncoder;
  public AnalogInput rollHomeEncoder;

  public CANSparkMaxSendable pitch;
  public RelativeEncoder pitchEncoder;
  public AnalogInput pitchHomeEncoder;
  /** Creates a new ArmSubsystem. */
  public CannonSubsystem() {
    setupMotors();
    cannonExtendMechanism = new CannonExtendMechanism(extend);
    cannonElevateMechanism = new CannonElevateMechanism(elevation);
    cannonRollMechanism = new CannonRollMechanism(roll);
    cannonPitchMechanism = new CannonPitchMechanism(pitch);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    cannonExtendMechanism.periodic();
    cannonElevateMechanism.periodic();
    cannonRollMechanism.periodic();
    cannonPitchMechanism.periodic();
    SmartDashboard.putNumber("elevate encoder distance", elevateEncoder.getDistance());
  }

  public void setLength(double length) {
    cannonExtendMechanism.setLength(length);
  }

  public void setHeight(double height) {
    cannonElevateMechanism.setHeight(height);
  }

  public void setRoll(double roll) {
    cannonRollMechanism.setRoll(roll);
  }

  public void setPitch(double pitch)
  {
    cannonPitchMechanism.setPitch(pitch);
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

    if (canDeviceFinder.isDevicePresent(CANDeviceType.SPARK_MAX, 11, "Roll") || shouldMakeAllCANDevices) {
      roll = new CANSparkMaxSendable(11, MotorType.kBrushless);
      addChild("roll", roll);
    }

    if(canDeviceFinder.isDevicePresent(CANDeviceType.SPARK_MAX, 12, "Pitch") || shouldMakeAllCANDevices) {
      pitch = new CANSparkMaxSendable(12, MotorType.kBrushless);
      addChild("pitch", pitch);
    }

    elevateEncoder = new Encoder(1, 2);
    addChild("elevateEncoder", elevateEncoder);
    elevateEncoder.setDistancePerPulse(-360/256.0);


  }
}

































































//:D