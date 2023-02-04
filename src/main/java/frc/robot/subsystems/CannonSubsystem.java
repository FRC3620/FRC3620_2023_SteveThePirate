// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.usfirst.frc3620.misc.CANDeviceFinder;
import org.usfirst.frc3620.misc.CANDeviceType;
import org.usfirst.frc3620.misc.CANSparkMaxSendable;

import com.revrobotics.AnalogInput;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;


public class CannonSubsystem extends SubsystemBase {
  public CannonExtendMechanism cannonExtendMechanism;
  
  public CANSparkMaxSendable elevation;
  public RelativeEncoder elevationEncoder;
  public AnalogInput elevationHomeEncoder;

  public CANSparkMaxSendable extend;
  public RelativeEncoder extendEncoder;
  public AnalogInput extendHomeEncoder;
  /** Creates a new ArmSubsystem. */
  public CannonSubsystem() {
    setupMotors();
    cannonExtendMechanism = new CannonExtendMechanism(extend);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    cannonExtendMechanism.periodic();
  }

  public void setLength(double length) {
    cannonExtendMechanism.setLength(length);
  }

  //Sets length of arm for movement. (Endgame?)
  public void setTravel() {
    
  }

  void setupMotors() {
    CANDeviceFinder canDeviceFinder = RobotContainer.canDeviceFinder;
		boolean shouldMakeAllCANDevices = RobotContainer.shouldMakeAllCANDevices();

    canDeviceFinder.isDevicePresent(CANDeviceType.SPARK_MAX, 9, "Elevation");
		elevation = new CANSparkMaxSendable(9, MotorType.kBrushless);

		canDeviceFinder.isDevicePresent(CANDeviceType.SPARK_MAX, 10, "Extend");
		extend = new CANSparkMaxSendable(10, MotorType.kBrushless);

  }
}
