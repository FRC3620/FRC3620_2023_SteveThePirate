// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.usfirst.frc3620.misc.CANSparkMaxSendable;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CannonClawMechanism {
  /** Creates a new ClawMechanism. */
  boolean encoderIsValid = false;
  Timer calibrationTimer;
  CANSparkMaxSendable motor;
  RelativeEncoder encoder;
  SparkMaxPIDController PID = null;
  
  Double requestedPositionWhileCalibrating = null;
  Double requestedPosition = null;

  final String name = "Claw";

  public CannonClawMechanism(CANSparkMaxSendable motor) {
    this.motor = motor;
    if (motor != null) {
      this.encoder = motor.getEncoder();
    }

    if (encoder != null) {
      encoder.setPositionConversionFactor(65/2.3);
      //encoder.setVelocityConversionFactor(1);
    }
  }

  public void periodic() {
    // This method will be called once per scheduler run
    if (motor != null) {
      SmartDashboard.putNumber(name + ".motor_current",  motor.getOutputCurrent());
      SmartDashboard.putNumber(name + ".power", motor.getAppliedOutput());

      if (encoder != null) {
        double elevateSpeed = encoder.getVelocity();
        SmartDashboard.putNumber(name + ".speed", elevateSpeed);
        // SmartDashboard.putNumber(name + ".velocityConversionFactor", encoder.getVelocityConversionFactor());

        /*if(Robot.getCurrentRobotMode() == RobotMode.TELEOP || Robot.getCurrentRobotMode() == RobotMode.AUTONOMOUS){
          if (!encoderIsValid) {
            moveClawCannon(0.015);
          
            if (calibrationTimer == null) {
              calibrationTimer = new Timer();
              calibrationTimer.reset();
              calibrationTimer.start();
            } else {
              if (calibrationTimer.get() > 0.75){
                if (Math.abs(elevateSpeed) < 15) {
                  encoderIsValid = true;
                  moveClawCannon(0.0);
                  encoder.setPosition(65);
                  
                  if (requestedPositionWhileCalibrating != null) {
                    setPosition(requestedPositionWhileCalibrating);
                    requestedPositionWhileCalibrating = null;
                  }
                }
              }
            }
          }
        } else {
          calibrationTimer = null; // start over!
        }*/
    }
  }
}

  /**
   * Sets the cannon to extend the arm to 'length' inches. Increasing
   * length is a longer arm.
   * "Extend" motor.*/

  public void setClawPower(double speed) {
    if (motor != null) {
      motor.set(speed);
    }
  }

  public double getClawPower() {
    if (encoder == null) return 0;
    return encoder.getVelocity();
  }
}