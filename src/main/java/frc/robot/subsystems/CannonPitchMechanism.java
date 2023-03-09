package frc.robot.subsystems;

import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.logger.EventLogging.Level;
import org.usfirst.frc3620.misc.CANSparkMaxSendable;
import org.usfirst.frc3620.misc.RobotMode;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CannonSubsystem;

public class CannonPitchMechanism  {
  Logger logger = EventLogging.getLogger(getClass(), Level.INFO);

  boolean encoderIsValid = false;
  Timer calibrationTimer;
  CANSparkMaxSendable motor;
  RelativeEncoder motorEncoder;
  Encoder pitchEncoder;

  double clampPitchforCommand = 0;
  
  Double requestedPositionWhileCalibrating = null;
  Double requestedPosition = null;

  private static final double kP = 0.004; //0.0025
  private static final double kI = 0;
  private static final double kD = 0;

  private final PIDController m_pidController = new PIDController(kP, kI, kD);

  double pitchOffset;
  CannonSubsystem cannonSubsystem;


  final String name = "Pitch";

  public CannonPitchMechanism(CANSparkMaxSendable motor, Encoder pitchEncoder, RelativeEncoder motorEncoder) {
    this.motor = motor;
    this.pitchEncoder = pitchEncoder;
    if (pitchEncoder != null) pitchEncoder.setDistancePerPulse(-360/256.0);
    this.motorEncoder = motorEncoder;
  }

  public void periodic() {
    if(cannonSubsystem == null){
      cannonSubsystem = RobotContainer.cannonSubsystem;
    }
    SmartDashboard.putBoolean(name + ".calibrated",  encoderIsValid);
    // This method will be called once per scheduler run
    if (motor != null) {
      SmartDashboard.putNumber(name + ".motor_current",  motor.getOutputCurrent());
      SmartDashboard.putNumber(name + ".power", motor.getAppliedOutput());

      if (pitchEncoder != null) {
        SmartDashboard.putNumber(name + ".speed", pitchEncoder.getRate());
        SmartDashboard.putNumber(name + ".position", getCurrentPitch());
        // SmartDashboard.putNumber(name + ".velocityConversionFactor", encoder.getVelocityConversionFactor());
        SmartDashboard.putNumber(name + ".motor_position", motorEncoder.getPosition());

        if(Robot.getCurrentRobotMode() == RobotMode.TELEOP || Robot.getCurrentRobotMode() == RobotMode.AUTONOMOUS){
          if (!encoderIsValid) {
            pitchCannon(-0.075);
          
            if (calibrationTimer == null) {
              calibrationTimer = new Timer();
              calibrationTimer.reset();
              calibrationTimer.start();
            } else {
              if (calibrationTimer.get() > 0.75){
                if (Math.abs(pitchEncoder.getRate()) < 15) {
                  encoderIsValid = true;
                  pitchCannon(0.0);
                  pitchOffset = pitchEncoder.getDistance() + 130;
                  setPitch(-130); // WHY? we setPitch down below?
                  motorEncoder.setPosition(-130);
                  
                  if (requestedPositionWhileCalibrating != null) {
                    setPitch(requestedPositionWhileCalibrating);
                    requestedPositionWhileCalibrating = null;
                  } else {
                    setPitch(pitchEncoder.getDistance() - pitchOffset); // is this right?
                  }
                }
              }
            }
          } else {
            double minPitch = (cannonSubsystem.getCurrentElevation() < -5) ? -10 : -130;
            double maxPitch = (cannonSubsystem.getCurrentElevation() > 75) ? -30 : 10;
            double clampPitch = MathUtil.clamp(requestedPosition, minPitch, maxPitch);
            SmartDashboard.putNumber(name + ".clampPitch", clampPitch);
            m_pidController.setSetpoint(clampPitch);
            clampPitchforCommand = clampPitch;


            double motorPower = m_pidController.calculate(getCurrentPitch());
            motorPower = MathUtil.clamp(motorPower, -0.5, 0.5);
            motor.set(motorPower);
          }
        } else {
          calibrationTimer = null; // start over!
        }
    }
  }
}

  /**
   * Sets the cannon to angle the wrist to 'pitch' degrees. 0 is perpendicular to
   * the end of the arm. Negative is more towards the front bumper.
   * @param pitch
   */
  public void setPitch(double pitch) {
    SmartDashboard.putNumber(name + ".requestedPitch", pitch);
    if (encoderIsValid) {
      requestedPosition = pitch;
    } else {
      requestedPositionWhileCalibrating = pitch;
    }
  }

  public void pitchCannon(double speed) {
      motor.set(speed);
  }

  public double getCurrentPitch() {
    if (pitchEncoder != null) {
      return pitchEncoder.getDistance() - pitchOffset;
    }
    return 0;
  }

  public double getRequestedPitch() {
    return requestedPosition;
  }

  public double getClampedPitch() {
    return clampPitchforCommand;
  }

  public void recalibrataePitch(boolean forward) {
    if (forward) {
      pitchOffset = pitchEncoder.getDistance() + 130;
      setPitch(-130);
    } else {
      pitchOffset = pitchEncoder.getDistance() - 20;
      setPitch(20);
    }
  }
}
