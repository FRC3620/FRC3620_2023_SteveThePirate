// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.logger.EventLogging.Level;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CannonSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class AutoLevelingCommand extends CommandBase {
  AHRS ahrs; 
  private DriveSubsystem driveSubsystem;
  private CannonSubsystem cannonSubsystem;
  double pitch;
  Logger logger = EventLogging.getLogger(getClass(), Level.INFO);


  enum MyState {
    LEVEL, TILTED, COUNTER, DONE
  } 
  
  MyState myState;
  
  /** Creates a new AutoLevelingCommand. */
  public AutoLevelingCommand(DriveSubsystem driveSubsystem, CannonSubsystem cannonSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSubsystem = driveSubsystem;
    this.cannonSubsystem = cannonSubsystem;
    addRequirements(driveSubsystem);
    myState = MyState.LEVEL;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveSubsystem.setDriveToBrake();

    cannonSubsystem.setPitch(-117);
    cannonSubsystem.setElevation(30);
    cannonSubsystem.setExtension(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pitch = RobotContainer.navigationSubsystem.getPitch();

    if(myState == MyState.LEVEL){
      //drive
      driveSubsystem.autoDrive(0, .3, 0);
      if(pitch < -13) {
        logger.info("switching to tilted, pitch = {}", pitch);
        myState = MyState.TILTED;
      }
    }
    
    if(myState == MyState.TILTED){
      //drive
      driveSubsystem.autoDrive(0, .1, 0);
     if(pitch > -10 && pitch < 1){
      logger.info("switching to counter, pitch = {}", pitch);
       myState = MyState.COUNTER;
      }
    }

    if(myState == MyState.COUNTER){
      if(pitch < 10){
        driveSubsystem.autoDrive(0, -.2, 0);
      } else {
        driveSubsystem.stopDrive();
        logger.info("switching to done, pitch = {}", pitch);
        myState = MyState.DONE;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    myState = MyState.LEVEL;
    driveSubsystem.setDriveToCoast();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(myState == MyState.DONE){
      return true;
    }
    return false;
  }
}
