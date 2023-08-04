// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.usfirst.frc3620.misc.CANSparkMaxSendable;
import org.usfirst.frc3620.misc.RobotMode;
import org.usfirst.frc3620.misc.SwerveCalculator;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class CannonElevateMechanism  {
  CANSparkMaxSendable motor;
  CANSparkMaxSendable motor2;
  AnalogInput elevateEncoder;
  RelativeEncoder motorEncoder;

  int elevateEncoderValueAt90Degrees;

  private static final double kP = 0.032;  //0.016
  private static final double kI = 0.002;  //0
  private static final double kD = 0.0005;   //0
  //private static final double pidTolerance = 30;

  private final PIDController m_pidController = new PIDController(kP, kI, kD);

  double elevationOffset;
  
  double requestedPosition = 90;

  final String name = "Elevate";

  public CannonElevateMechanism(CANSparkMaxSendable motor, AnalogInput elevateEncoder, RelativeEncoder motorEncoder, CANSparkMaxSendable motor2) {
    this.motor = motor;
    this.motor2 = motor2;
    this.elevateEncoder = elevateEncoder;
    this.motorEncoder = motorEncoder;
    this.elevateEncoderValueAt90Degrees = RobotContainer.robotParameters.getElevationEncoderValueAt90Degrees();

    setElevation(90);   // start straight up!

    if (motorEncoder != null) {
      double e = getCurrentElevation();
      motorEncoder.setPosition(e); // syncronize once
    }

    if (motor != null && motor2 != null) {
      motor2.follow(motor);
    }
  }

  public void periodic() {
    // This method will be called once per scheduler run
    if (motor != null) {
      if (Robot.getCurrentRobotMode() == RobotMode.TELEOP || Robot.getCurrentRobotMode() == RobotMode.AUTONOMOUS) {
        double currentElevation = getCurrentElevation();
        double motorPower = m_pidController.calculate(currentElevation); // negative power if cannon is moving towards front
        double minPower = -0.75;
        double maxPower = 0.95;

        //m_pidController.setTolerance(pidTolerance);

        /*if(Math.abs(requestedPosition - elevateEncoder.getValue()) > 30){
          /*kP = 0.04;  //0.016
          kI = 0.001;  //0
          kD = 0.02;   //0
          PIDController.setPID(0.04, 0.001, 0.02);
        } else {
          kP = 0.016;
          kI = 0;
          kD = 0;
        }*/
        //DO NOT UNCOMMENT THIS IT WILL BREAK THINGS
        //m_pidController = new PIDController(kP, kI, kD);

        // TODO put this back
        // motorPower = MathUtil.clamp(motorPower, -0.6, 0.87);
        if (currentElevation < 0) {
          // limit power to -0.25 instead of -0.75 if the cannon is past 0 so we don't ram it into the ground
          motorPower = MathUtil.clamp(motorPower, -0.25, 0.95);
        }
        else if (currentElevation < 60) {
          // limit power in between -0.75 and 0.95 if the cannon is 0-60 (front)
          motorPower = MathUtil.clamp(motorPower, -0.75, 0.95);
        }
        else if (currentElevation < 120) {
          // we want to limit power in between -0.75 and 0.75 if the cannon is in between 60 and 120
          motorPower = MathUtil.clamp(motorPower, -0.75, 0.75);
        }
        else if (currentElevation < 180) {
          // limit power in between -0.95 and 0.75 if the cannon is past 120 (back)
          motorPower = MathUtil.clamp(motorPower, -0.95, 0.75);
        }
        else {
          // limit power to 0.25 at the max if the cannon is past 180 so we don't ram it into the ground
          motorPower = MathUtil.clamp(motorPower, -0.95, 0.25);
        }
          elevateCannon(motorPower);
      }

      SmartDashboard.putNumber(name + ".motor_current", motor.getOutputCurrent());
      SmartDashboard.putNumber(name + ".power", motor.getAppliedOutput());
      SmartDashboard.putNumber(name + ".temperature", motor.getMotorTemperature());

      SmartDashboard.putNumber(name + ".currentEncoderValue", elevateEncoder.getValue());
      SmartDashboard.putNumber(name + ".currentPosition", getCurrentElevation());
      SmartDashboard.putNumber(name + ".currentMotorPosition", motorEncoder.getPosition());
    }
    
    if (motor2 != null) {
      SmartDashboard.putNumber(name + "2.motor_current", motor2.getOutputCurrent());
      SmartDashboard.putNumber(name + "2.power", motor2.getAppliedOutput());
      SmartDashboard.putNumber(name + "2.temperature", motor2.getMotorTemperature());
    }
  }

  /**
   * Sets the cannon to elevate the arm to 'elevation' degrees. 0 is horizontal
   * in front. Increasing angle is up (90 is straight up).
   * @param elevation
   */
  public void setElevation(double elevation) {
    elevation = MathUtil.clamp(elevation, -45, 200);
    SmartDashboard.putNumber(name + ".requestedElevation", elevation);
    requestedPosition = elevation;

    m_pidController.setSetpoint(elevation);
  }

  public double getRequestedElevation() {
    return requestedPosition;
  }

  public void elevateCannon(double power) {
    motor.set(power);
  }

  public double getCurrentElevation(){
    if (elevateEncoder == null) return 0;
		int elevationEncoderValue = elevateEncoder.getValue();
    // converting heading from tics (ranging from 0 to 4095) to degrees
		double elevation = (elevationEncoderValue - elevateEncoderValueAt90Degrees)*(360.0/4096.0) + 90;
    // get it into the -180..180 range
    elevation = SwerveCalculator.normalizeAngle(elevation);
    // get it into the -90..270 range
    if (elevation < -90){
      elevation = elevation + 360;
    }
		return elevation;
	}
}

//:D