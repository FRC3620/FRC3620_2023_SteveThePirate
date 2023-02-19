// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Base64.Encoder;

import org.usfirst.frc3620.misc.CANSparkMaxSendable;
import org.usfirst.frc3620.misc.RobotMode;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class CannonExtendMechanism  {
  /** Creates a new ExtendSubSubsystem. */
  boolean encoderIsValid = false;
  Timer calibrationTimer;
  CANSparkMaxSendable motor;
  RelativeEncoder encoder;


  SparkMaxPIDController PID = null;
  
  Double requestedPositionWhileCalibrating = null;
  Double requestedPosition = null;

  final String name = "Extension";





  public CannonExtendMechanism(CANSparkMaxSendable motor) {
    this.motor = motor;
    if (motor != null) {
      this.encoder = motor.getEncoder();

      PID = motor.getPIDController();

      // set up PID for turretPID here
      PID.setP(0.2);   //0.1
      PID.setI(0.0);     //0.0
      PID.setD(0);    //10
      PID.setFF(0.0);      //0.0

      PID.setOutputRange(-0.2, 0.2);
    }

    if (encoder != null) {
      encoder.setPositionConversionFactor((30.25-4.125)/(16.4-4.8));
      //encoder.setVelocityConversionFactor(1);
    }
  }

  public void periodic() {
    
    SmartDashboard.putBoolean(name + ".calibrated",  encoderIsValid);
    // This method will be called once per scheduler run
    if (motor != null) {
      SmartDashboard.putNumber(name + ".current",  motor.getOutputCurrent());
      SmartDashboard.putNumber(name + ".power", motor.getAppliedOutput());

      if (encoder != null) {
        double extendSpeed = encoder.getVelocity();
        double extendPosition = encoder.getPosition();
        SmartDashboard.putNumber(name + ".speed", extendSpeed);
        SmartDashboard.putNumber(name + ".position", extendPosition);
        // SmartDashboard.putNumber(name + ".velocityConversionFactor", encoder.getVelocityConversionFactor());

        if(Robot.getCurrentRobotMode() == RobotMode.TELEOP || Robot.getCurrentRobotMode() == RobotMode.AUTONOMOUS){
          if (!encoderIsValid) {
            extendCannon(-0.03);
          
            if (calibrationTimer == null) {
              calibrationTimer = new Timer();
              calibrationTimer.reset();
              calibrationTimer.start();
            } else {
              if (calibrationTimer.get() > 0.5){
                if (Math.abs(extendSpeed) < 2) {
                  encoderIsValid = true;
                  extendCannon(0.0);
                  encoder.setPosition(0.0);
                  if (requestedPositionWhileCalibrating != null) {
                    setLength(requestedPositionWhileCalibrating);
                    requestedPositionWhileCalibrating = null;
                  }
                }
              }
            }
          }
        } else {
          calibrationTimer = null; // start over!
        }
    }
  }
}

  /**
   * Sets the cannon to extend the arm to 'length' inches. Increasing
   * length is a longer arm.
   * "Extend" motor.
   * @param length
   */
  public void setLength(double length) {
    length = MathUtil.clamp(length, 0, 18);
    SmartDashboard.putNumber(name + ".requestedLength", length);
    requestedPosition = length;
    if (encoderIsValid) {
      PID.setReference(length, ControlType.kPosition);
    } else {
      requestedPositionWhileCalibrating = length;
    }
  }

  public void extendCannon(double speed) {
      motor.set(speed);
  }

  public double getExtension() {
    return requestedPosition;
  }
}
