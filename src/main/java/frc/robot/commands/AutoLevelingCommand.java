// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.logger.IFastDataLogger;
import org.usfirst.frc3620.logger.EventLogging.Level;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ILevelingDataSource;
import frc.robot.LevelingDataLogger;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CannonSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class AutoLevelingCommand extends CommandBase implements ILevelingDataSource {
  AHRS ahrs; 
  private DriveSubsystem driveSubsystem;
  private CannonSubsystem cannonSubsystem;
  double pitch;
  Logger logger = EventLogging.getLogger(getClass(), Level.INFO);

  enum LevelingState {
    LEVEL, TILTED, COUNTER, DONE
  }

  LevelingState myState;

  final static boolean doLog = true;
  IFastDataLogger levelingDataLogger = null;
  
  /** Creates a new AutoLevelingCommand. */
  public AutoLevelingCommand(DriveSubsystem driveSubsystem, CannonSubsystem cannonSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSubsystem = driveSubsystem;
    this.cannonSubsystem = cannonSubsystem;
    addRequirements(driveSubsystem);
    myState = LevelingState.LEVEL;

    if (doLog) levelingDataLogger = LevelingDataLogger.getDataLogger(getClass().getSimpleName(), this);
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

    if(myState == LevelingState.LEVEL){
      //drive
      driveSubsystem.autoDrive(0, .3, 0);
      if(pitch < -13) {
        logger.info("switching to tilted, pitch = {}", pitch);
        myState = LevelingState.TILTED;
      }
    }
    
    if(myState == LevelingState.TILTED){
      //drive
      driveSubsystem.autoDrive(0, .1, 0);
     if(pitch > -10 && pitch < 1){
      logger.info("switching to counter, pitch = {}", pitch);
       myState = LevelingState.COUNTER;
      }
    }

    if(myState == LevelingState.COUNTER){
      if(pitch < 10){
        driveSubsystem.autoDrive(0, -.2, 0);
      } else {
        driveSubsystem.stopDrive();
        logger.info("switching to done, pitch = {}", pitch);
        myState = LevelingState.DONE;
      }
    }

    if (levelingDataLogger != null) levelingDataLogger.update();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    myState = LevelingState.LEVEL;
    if (levelingDataLogger != null) levelingDataLogger.done();
    driveSubsystem.setDriveToCoast();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(myState == LevelingState.DONE) {
      return true;
    }
    return false;
  }

  @Override
  public LevelingData getLevelingData() {
    return new LevelingData("" + myState, pitch);
  }
}
