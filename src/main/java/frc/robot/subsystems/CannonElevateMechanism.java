// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.usfirst.frc3620.misc.CANSparkMaxSendable;
import org.usfirst.frc3620.misc.RobotMode;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class CannonElevateMechanism  {
  /** Creates a new ExtendSubSubsystem. */
  boolean encoderIsValid = false;
  Timer calibrationTimer;
  CANSparkMaxSendable motor;
  RelativeEncoder encoder;
  SparkMaxPIDController PID = null;
  
  Double requestedPositionWhileCalibrating = null;
  Double requestedPosition = null;

  final String name = "Elevate";

  public CannonElevateMechanism(CANSparkMaxSendable motor) {
    this.motor = motor;
    if (motor != null) {
      this.encoder = motor.getEncoder();

      PID = motor.getPIDController();

      // set up PID for turretPID here
      PID.setP(0.1);   //0.1
      PID.setI(0.0);     //0.0
      PID.setD(10);    //10
      PID.setFF(0.0);      //0.0

      PID.setOutputRange(-0.1, 0.1);
    }

    if (encoder != null) {
      //encoder.setPositionConversionFactor(90.0/115.0);
      //encoder.setVelocityConversionFactor(1);
    }
  }

  public void periodic() {
    SmartDashboard.putString("IS THIS CALLED ELEVATE?????!?!?!", name);
    SmartDashboard.putBoolean(name + ".calibrated",  encoderIsValid);
    // This method will be called once per scheduler run
    if (motor != null) {
      SmartDashboard.putNumber(name + ".current",  motor.getOutputCurrent());
      SmartDashboard.putNumber(name + ".power", motor.getAppliedOutput());

      if (encoder != null) {
        double elevateSpeed = encoder.getVelocity();
        double elevatePosition = encoder.getPosition();
        SmartDashboard.putNumber(name + ".speed", elevateSpeed);
        SmartDashboard.putNumber(name + ".position", elevatePosition);
        // SmartDashboard.putNumber(name + ".velocityConversionFactor", encoder.getVelocityConversionFactor());

        if(Robot.getCurrentRobotMode() == RobotMode.TELEOP || Robot.getCurrentRobotMode() == RobotMode.AUTONOMOUS){
          if (!encoderIsValid) {
            elevateCannon(0.03);
          
            if (calibrationTimer == null) {
              calibrationTimer = new Timer();
              calibrationTimer.reset();
              calibrationTimer.start();
            } else {
              if (calibrationTimer.get() > 0.5){
                if (Math.abs(elevateSpeed) < 2) {
                  encoderIsValid = true;
                  elevateCannon(0.0);
                  encoder.setPosition(0.0);
                  if (requestedPositionWhileCalibrating != null) {
                    setHeight(requestedPositionWhileCalibrating);
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
   * @param height
   */
  public void setHeight(double height) {
    if(height < 0) {
      height = 0;
    }
    if(height > 24) {
      height = 24;
    }
    SmartDashboard.putNumber(name + ".requestedHeight", height);
    requestedPosition = height;
    if (encoderIsValid) {
      PID.setReference(height, ControlType.kPosition);
    } else {
      requestedPositionWhileCalibrating = height;
    }
  }

  public void elevateCannon(double speed) {
      motor.set(speed);
  }
}
