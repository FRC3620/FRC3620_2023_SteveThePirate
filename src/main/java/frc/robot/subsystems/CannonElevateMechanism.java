// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.usfirst.frc3620.misc.CANSparkMaxSendable;
import org.usfirst.frc3620.misc.RobotMode;
import org.usfirst.frc3620.misc.SwerveCalculator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class CannonElevateMechanism  {
  CANSparkMaxSendable motor;
  AnalogInput elevateEncoder;

  int elevateEncoderValueAt90Degrees;

  private static final double kP = 0.02;
  private static final double kI = 0;
  private static final double kD = 0;

  private final PIDController m_pidController = new PIDController(kP, kI, kD);

  double elevationOffset;
  
  double requestedPosition = 90;

  final String name = "Elevate";

  public CannonElevateMechanism(CANSparkMaxSendable motor, AnalogInput elevateEncoder) {
    this.motor = motor;
    this.elevateEncoder = elevateEncoder;
    this.elevateEncoderValueAt90Degrees = RobotContainer.robotParameters.getElevationEncoderValueAt90Degrees();

    setElevation(90);   // start straight up!
  }

  public void periodic() {
    // This method will be called once per scheduler run
    if (motor != null) {
      if (Robot.getCurrentRobotMode() == RobotMode.TELEOP || Robot.getCurrentRobotMode() == RobotMode.AUTONOMOUS) {
        double motorPower = m_pidController.calculate(getCurrentElevation());
        motorPower = MathUtil.clamp(motorPower, -0.4, 0.75);
        elevateCannon(motorPower);
      }
      
      SmartDashboard.putNumber(name + ".motor_current", motor.getOutputCurrent());
      SmartDashboard.putNumber(name + ".power", motor.getAppliedOutput());
      SmartDashboard.putNumber(name + ".temperature", motor.getMotorTemperature());

      SmartDashboard.putNumber(name + ".currentEncoderValue", elevateEncoder.getValue());
      SmartDashboard.putNumber(name + ".currentPosition", getCurrentElevation());
    }
  }

  /**
   * Sets the cannon to elevate the arm to 'elevation' degrees. 0 is horizontal
   * in front. Increasing angle is up (90 is straight up).
   * @param elevation
   */
  public void setElevation(double elevation) {
    elevation = MathUtil.clamp(elevation, -45, 125);
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
		return elevation;
	}
}

//:D