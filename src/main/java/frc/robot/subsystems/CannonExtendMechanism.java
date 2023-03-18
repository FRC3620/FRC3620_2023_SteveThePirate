package frc.robot.subsystems;

import java.util.Map;

import org.usfirst.frc3620.misc.CANSparkMaxSendable;
import org.usfirst.frc3620.misc.RobotMode;

import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class CannonExtendMechanism  {
  /** Creates a new ExtendSubSubsystem. */
  boolean encoderIsValid = false;
  Timer calibrationTimer;
  CANSparkMaxSendable motor;
  CANSparkMaxSendable motor2;
  RelativeEncoder encoder;

  SparkMaxPIDController PID = null;
  
  Double requestedPositionWhileCalibrating = null;
  double requestedPosition = 0;
  double adjustmentAddition = 0;
  double adjustedLength = requestedPosition;

  final String name = "Extension";

  public GenericEntry adjustemEntry = Shuffleboard.getTab("Match")
    .add(name + ".adjustmentSlider", 0)
    .withWidget(BuiltInWidgets.kNumberSlider)
    .withProperties(Map.of("min", -5, "max", 5))
    .getEntry();

  public CannonExtendMechanism(CANSparkMaxSendable motor, CANSparkMaxSendable motor2) {
    this.motor = motor;
    if (motor != null) {
      this.encoder = motor.getEncoder();

      PID = motor.getPIDController();

      // set up PID for turretPID here
      PID.setP(0.075);   //0.1
      PID.setI(0.0);     //0.0
      PID.setD(0);    //10
      PID.setFF(0.0);      //0.0

      PID.setOutputRange(-0.4, 0.95);
    }

    if (motor != null && motor2 != null) {
      motor2.follow(motor);
    }

    if (encoder != null) {
      encoder.setPositionConversionFactor(18.75/30.4);
      //encoder.setVelocityConversionFactor(1);
    }
  }
  

  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putBoolean(name + ".calibrated", encoderIsValid);

    if (motor2 != null) {
      SmartDashboard.putNumber(name + "2.motor_current", motor2.getOutputCurrent());
      SmartDashboard.putNumber(name + "2.power", motor2.getAppliedOutput());
      SmartDashboard.putNumber(name + "2.temperature", motor2.getMotorTemperature());
    }

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
    adjustmentAddition = adjustemEntry.getDouble(0);
    adjustedLength = length + adjustmentAddition;
    adjustedLength = MathUtil.clamp(adjustedLength, 0, 36);
    SmartDashboard.putNumber(name + ".rawRequestedLength", length);
    SmartDashboard.putNumber(name + ".requestedLength", adjustedLength);
    
    requestedPosition = adjustedLength;
    if (encoderIsValid) {
      PID.setReference(adjustedLength, ControlType.kPosition);
    } else {
      requestedPositionWhileCalibrating = adjustedLength;
    }
  }

  public void extendCannon(double power) {
    motor.set(power);
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

  public double getAdjustedRequestedExtension() {
    return adjustedLength;
  }
}