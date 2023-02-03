// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.usfirst.frc3620.misc.CANDeviceFinder;
import org.usfirst.frc3620.misc.CANDeviceType;
import org.usfirst.frc3620.misc.CANSparkMaxSendable;

import com.revrobotics.AnalogInput;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;


public class CannonSubsystem extends SubsystemBase {
  public CANSparkMaxSendable elevation;
  public RelativeEncoder elevationEncoder;
  public AnalogInput elevationHomeEncoder;

  public CANSparkMaxSendable extend;
  public RelativeEncoder extendEncoder;
  public AnalogInput extendHomeEncoder;
  /** Creates a new ArmSubsystem. */
  public CannonSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Sets the cannon to extend the arm to 'length' inches. Increasing
   * length is a longer arm.
   * "Extend" motor.
   * @param length
   */
  public void setLength(double length) {

  }
  
  /**
   * Sets cannon angle relative to the robot.
   * "Elevation" motor
   * @param angle
   */
  public void setAngle(double angle) {

  }
  
  /**
   * Sets the roll of the wrist.
   * @param rollAngle
   */
  public void setRoll(double rollAngle) {

  }

  /**
   * Sets the pitch of the wrist.
   * @param pitchAngle
   */
  public void setPitch(double pitchAngle) {

  }
  //Sets length of arm for movement. (Endgame?)
  public void setTravel() {

  }

  void setupMotors() {
    CANDeviceFinder canDeviceFinder = RobotContainer.canDeviceFinder;
		boolean shouldMakeAllCANDevices = RobotContainer.shouldMakeAllCANDevices();

    canDeviceFinder.isDevicePresent(CANDeviceType.SPARK_MAX, 9, "Elevation");
		elevation = new CANSparkMaxSendable(9, null);

		canDeviceFinder.isDevicePresent(CANDeviceType.SPARK_MAX, 10, "Extend");
		elevation = new CANSparkMaxSendable(10, null);

  }
}
