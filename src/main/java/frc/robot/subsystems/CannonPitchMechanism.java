package frc.robot.subsystems;

import org.usfirst.frc3620.misc.CANSparkMaxSendable;
import org.usfirst.frc3620.misc.RobotMode;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.CannonLocation;
import frc.robot.Robot;

public class CannonPitchMechanism  {
  /** Creates a new ExtendSubSubsystem. */
  boolean encoderIsValid = false;
  Timer calibrationTimer;
  CANSparkMaxSendable motor;
  RelativeEncoder encoder;
  SparkMaxPIDController PID = null;
  
  Double requestedPositionWhileCalibrating = null;
  Double requestedPosition = null;

  final String name = "Pitch";

  public CannonPitchMechanism(CANSparkMaxSendable motor) {
    this.motor = motor;
    if (motor != null) {
      this.encoder = motor.getEncoder();

      PID = motor.getPIDController();

      // set up PID for turretPID here
      PID.setP(0.0025);   //0.1
      PID.setI(0.0);     //0.0
      PID.setD(0.0);    //10
      PID.setFF(0.0);      //0.0

      PID.setOutputRange(-0.4, 0.4);
    }

    if (encoder != null) {
      //calculated by two positions difference of angle over difference in encoder value
      encoder.setPositionConversionFactor(191/6.05);
      //encoder.setPositionConversionFactor(1);
      //encoder.setVelocityConversionFactor(1);
    }
  }

  public void periodic() {
    SmartDashboard.putBoolean(name + ".calibrated",  encoderIsValid);
    // This method will be called once per scheduler run
    if (motor != null) {
      SmartDashboard.putNumber(name + ".motor_current",  motor.getOutputCurrent());
      SmartDashboard.putNumber(name + ".power", motor.getAppliedOutput());

      if (encoder != null) {
        double elevateSpeed = encoder.getVelocity();
        double elevatePosition = encoder.getPosition();
        SmartDashboard.putNumber(name + ".speed", elevateSpeed);
        SmartDashboard.putNumber(name + ".position", elevatePosition);
        // SmartDashboard.putNumber(name + ".velocityConversionFactor", encoder.getVelocityConversionFactor());

        if(Robot.getCurrentRobotMode() == RobotMode.TELEOP || Robot.getCurrentRobotMode() == RobotMode.AUTONOMOUS){
          if (!encoderIsValid) {
            pitchCannon(-0.075);
          
            if (calibrationTimer == null) {
              calibrationTimer = new Timer();
              calibrationTimer.reset();
              calibrationTimer.start();
            } else {
              if (calibrationTimer.get() > 0.75){
                if (Math.abs(elevateSpeed) < 15) {
                  encoderIsValid = true;
                  pitchCannon(0.0);
                  encoder.setPosition(-140);
                  setPitch(-140);
                  
                  if (requestedPositionWhileCalibrating != null) {
                    setPitch(requestedPositionWhileCalibrating);
                    requestedPositionWhileCalibrating = null;
                  } else {
                    setPitch(encoder.getPosition());
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
   * @param pitch
   */
  public void setPitch(double pitch) {
    pitch = MathUtil.clamp(pitch, -150, 20);
    SmartDashboard.putNumber(name + ".requestedHeight", pitch);
    requestedPosition = pitch;
    if (encoderIsValid) {
      PID.setReference(pitch, ControlType.kPosition);
    } else {
      requestedPositionWhileCalibrating = pitch;
    }
  }

  public void pitchCannon(double speed) {
      motor.set(speed);
  }

  public double getCurrentPitch() {
    if (encoder != null) {
      return encoder.getPosition();
    }
    return 0;
  }

  public double getRequestedPitch() {
    return requestedPosition;
  }
}
