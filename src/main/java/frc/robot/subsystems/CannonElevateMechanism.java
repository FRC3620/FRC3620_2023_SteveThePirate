// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.concurrent.ThreadPoolExecutor.DiscardOldestPolicy;

import org.usfirst.frc3620.misc.CANSparkMaxSendable;
import org.usfirst.frc3620.misc.RobotMode;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class CannonElevateMechanism  {
  /** Creates a new ExtendSubSubsystem. */
  boolean encoderIsValid = false;
  CANSparkMaxSendable motor;
  Encoder elevateEncoder;
  DigitalInput homeSwitch;
  DigitalInput notHomeSwitch;
  private static final double kP = 0.02;
  private static final double kI = 0;
  private static final double kD = 0;

  private final PIDController m_pidController = new PIDController(kP, kI, kD);

  double elevationOffset;
  
  Double requestedPositionWhileCalibrating = null;
  double requestedPosition = 0;

  final String name = "Elevate";

  public CannonElevateMechanism(CANSparkMaxSendable motor, Encoder elevateEncoder, DigitalInput homeSwitch, DigitalInput notHomeSwitch) {
    this.motor = motor;
    this.elevateEncoder = elevateEncoder;
    this.homeSwitch = homeSwitch;
    this.notHomeSwitch = notHomeSwitch;
    elevateEncoder.setDistancePerPulse(-360/256.0);

  }

  public void periodic() {
    SmartDashboard.putNumber("elevate encoder rate", elevateEncoder.getRate());
    SmartDashboard.putBoolean(name + ".calibrated", encoderIsValid);
    // This method will be called once per scheduler run
    if (motor != null) {
      SmartDashboard.putNumber(name + ".current", motor.getOutputCurrent());
      SmartDashboard.putNumber(name + ".power", motor.getAppliedOutput());
      SmartDashboard.putNumber(name + ".temperature", motor.getMotorTemperature());

      double elevateSpeed = elevateEncoder.getRate();
      double elevatePosition = elevateEncoder.getDistance();
      SmartDashboard.putNumber(name + ".speed", elevateSpeed);
      SmartDashboard.putNumber(name + ".rawPosition", elevatePosition);
      SmartDashboard.putNumber(name + ".offsetPosition", getCurrentElevation());
      SmartDashboard.putBoolean(name + "Am I home?", amIHome());
      SmartDashboard.putBoolean(name + "Am I in front?", amIInFront());
      // SmartDashboard.putNumber(name + ".velocityConversionFactor",
      // encoder.getVelocityConversionFactor());

      if (Robot.getCurrentRobotMode() == RobotMode.TELEOP || Robot.getCurrentRobotMode() == RobotMode.AUTONOMOUS) {
        if (!encoderIsValid) {

          if (amIHome()) {
            encoderIsValid = true;
            elevateCannon(0.0);
            elevationOffset = -elevateEncoder.getDistance() + 90;
            setElevation(90);
            if (requestedPositionWhileCalibrating != null) {
              setElevation(requestedPositionWhileCalibrating);
              requestedPositionWhileCalibrating = null;
            }
          } else {
            if (amIInFront()) {
              elevateCannon(0.05);
            } else {
              elevateCannon(-0.05);
            }

          }
        } else {
          // encoder is valid
          double motorPower = m_pidController.calculate(getCurrentElevation());
          motorPower = MathUtil.clamp(motorPower, -0.4, 0.75);
          elevateCannon(motorPower);
        }

      }
    }
  }

  /**
   * Sets the cannon to elevate the arm to 'elevation' degrees. 0 is horizontal
   * in front. Increasing angle is up (90 is straight up).
   * @param elevation
   */
  public void setElevation(double elevation) {
    elevation = MathUtil.clamp(elevation, -25, 145);
    SmartDashboard.putNumber(name + ".requestedElevation", elevation);
    requestedPosition = elevation;
    if (encoderIsValid) {
      m_pidController.setSetpoint(elevation);
    } else {
      requestedPositionWhileCalibrating = elevation;
    }
  }

  public double getRequestedElevation() {
    return requestedPosition;
  }

  public void elevateCannon(double power) {
      motor.set(power);
  }

  public double getCurrentElevation(){
    return elevateEncoder.getDistance() + elevationOffset;
  }

  public boolean amIHome(){
    return !homeSwitch.get();
  }

  public boolean amIInFront(){
    return !notHomeSwitch.get();
  }
}





























//:D