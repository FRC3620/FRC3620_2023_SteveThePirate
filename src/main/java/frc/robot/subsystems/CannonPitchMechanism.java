package frc.robot.subsystems;

import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.logger.EventLogging.Level;
import org.usfirst.frc3620.misc.CANSparkMaxSendable;
import org.usfirst.frc3620.misc.RobotMode;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class CannonPitchMechanism  {
  Logger logger = EventLogging.getLogger(getClass(), Level.INFO);

  boolean encoderIsValid = false;
  Timer calibrationTimer;
  CANSparkMaxSendable motor;
  RelativeEncoder motorEncoder;
  Encoder pitchEncoder;

  double clampedPitch = 0;
  
  Double requestedPositionWhileCalibrating = null;
  double requestedPosition = 0;

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
                  logger.info("calibration completed");
                  encoderIsValid = true;
                  calibrationTimer = null;
                  pitchCannon(0.0);
                  pitchOffset = pitchEncoder.getDistance() + 130;
                  setPitch(-130); // WHY? we setPitch down below?
                  motorEncoder.setPosition(-130);
                  
                  if (requestedPositionWhileCalibrating != null) {
                    setPitch(requestedPositionWhileCalibrating);
                    requestedPositionWhileCalibrating = null;
                  } else {
                    setPitch(-130); // is this right?
                  }
                }
              }
            }
          } else {
            double maxPitch;
            double minPitch;
            double currentElevation = cannonSubsystem.getCurrentElevation();
            //double maxPitch = (cannonSubsystem.getCurrentElevation() > 75) ? -30 : 20; //TODO 17
            if (currentElevation < 75) {
              maxPitch = 20;
            }
            else if (currentElevation < 135) {
              maxPitch = -30;
            }
            else {
              maxPitch = 48;
            }

            // double minPitch = (cannonSubsystem.getCurrentElevation() < -5) ? -10 : -130;
            if (currentElevation < -5) {
              minPitch = -10;
            }
            else {
              minPitch = -130;
            }

            clampedPitch = MathUtil.clamp(requestedPosition, minPitch, maxPitch);
            SmartDashboard.putNumber(name + ".clampPitch", clampedPitch);
            m_pidController.setSetpoint(clampedPitch);

            double motorPower = m_pidController.calculate(motorEncoder.getPosition());
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
    return clampedPitch;
  }

  public void recalibrataePitch() {
    logger.info("recalibratae started");
    encoderIsValid = false;
  }
}
