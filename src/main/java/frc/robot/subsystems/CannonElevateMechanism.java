// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.usfirst.frc3620.misc.RobotMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class CannonElevateMechanism extends SubsystemBase {
  /** Creates a new ElevateSubSubsystem. */
  boolean encoderIsValid = false;
  Timer calibrationTimer;
  
  public CannonElevateMechanism() {}

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    /*if(Robot.getCurrentRobotMode() == RobotMode.TELEOP || Robot.getCurrentRobotMode() == RobotMode.AUTONOMOUS){
      if (!encoderIsValid) {
        turnTurret(0.06);

        if (calibrationTimer == null) {
          calibrationTimer = new Timer();
          calibrationTimer.reset();
          calibrationTimer.start();
        } else {
          if (calibrationTimer.get() > 0.5){
            if (Math.abs(turretSpeed) < 20) {
              encoderIsValid = true;
              turnTurret(0.0);
              turretEncoder.setPosition(270.0);
              if (requestedTurretPositionWhileCalibrating != null) {
                setTurretPosition(requestedTurretPositionWhileCalibrating);
                requestedTurretPositionWhileCalibrating = null;
              }
            }
          }
        }
      }
    } else {
      calibrationTimer = null; // start over!
    }*/
  }

  /**
   * Sets cannon angle relative to the robot.
   * "Elevation" motor
   * @param angle
   */
  public void setAngle(double angle) {

  }
}
