package frc.robot.subsystems;

import org.usfirst.frc3620.misc.CANSparkMaxSendable;
import org.usfirst.frc3620.misc.RobotMode;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class CannonExtendMechanism  {
  /** Creates a new ExtendSubSubsystem. */
  boolean encoderIsValid = false;
  Timer calibrationTimer;
  CANSparkMaxSendable motor;
  RelativeEncoder encoder;

  SparkMaxPIDController PID = null;
  
  Double requestedPositionWhileCalibrating = null;
  double requestedPosition = 0;

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
      encoder.setPositionConversionFactor(15/30.4);
      //encoder.setVelocityConversionFactor(1);
    }
  }

  public void periodic() {
    
    SmartDashboard.putBoolean(name + ".calibrated",  encoderIsValid);
    // This method will be called once per scheduler run
    if (motor != null) {
      SmartDashboard.putNumber(name + ".motor_current",  motor.getOutputCurrent());
      SmartDashboard.putNumber(name + ".power", motor.getAppliedOutput());
      SmartDashboard.putNumber(name + ".temperature", motor.getMotorTemperature());

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
                    setExtension(requestedPositionWhileCalibrating);
                    requestedPositionWhileCalibrating = null;
                  } else {
                    // this might try to extend arm while vertical, a big no-no!
                    // setLength(encoder.getPosition());
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
  public void setExtension(double length) {
    length = MathUtil.clamp(length, 0, 45);
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

  public void disable() {
    motor.stopMotor();
  }

  public double getRequestedExtension() {
    return requestedPosition;
  }

  public double getCurrentExtension() {
    if (encoder == null) return 0;
    return  encoder.getPosition();
  }
}
