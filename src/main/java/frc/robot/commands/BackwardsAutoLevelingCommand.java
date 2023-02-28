// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.usfirst.frc3620.logger.IFastDataLogger;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ILevelingDataSource;
import frc.robot.LevelingDataLogger;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CannonSubsystem;
import frc.robot.subsystems.DriveSubsystem;

public class BackwardsAutoLevelingCommand extends CommandBase implements ILevelingDataSource {
  AHRS ahrs; 
  private DriveSubsystem driveSubsystem;
  private CannonSubsystem cannonSubsystem;

  double pitch;
  double power;

  enum LevelingState {
    LEVEL, TILTED, COUNTER, DONE
  }

  LevelingState myState;

  final static boolean doLog = true;
  IFastDataLogger levelingDataLogger = null;  

  /** Creates a new AutoLevelingCommand. */
  public BackwardsAutoLevelingCommand(DriveSubsystem driveSubsystem, CannonSubsystem cannonSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSubsystem = driveSubsystem;
    this.cannonSubsystem = cannonSubsystem;
    addRequirements(driveSubsystem);
    myState = LevelingState.LEVEL;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveSubsystem.setDriveToBrake();

    cannonSubsystem.setPitch(-117);
    cannonSubsystem.setElevation(30);
    cannonSubsystem.setExtension(0);

    if (doLog) {
      levelingDataLogger = LevelingDataLogger.getDataLogger(getClass().getSimpleName(), this);
      levelingDataLogger.start();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    power = 0;
    pitch = RobotContainer.navigationSubsystem.getPitch();

    if(myState == LevelingState.LEVEL){
      power = -0.3;
      if(pitch > 13) {
        myState = LevelingState.TILTED;
      }
    }
    
    if(myState == LevelingState.TILTED){
      power = -0.1;
      if(pitch < 10 && pitch > -1){
        myState = LevelingState.COUNTER;
      }
    }

    if(myState == LevelingState.COUNTER){
      if(pitch > -10){
        power = 0.23;
      } else {
        power = 0;
        myState = LevelingState.DONE;
      }
    }

    if (power == 0) {
      driveSubsystem.stopDrive();
    } else {
      driveSubsystem.autoDrive(0, power, 0);
    }

    if (levelingDataLogger != null) levelingDataLogger.update();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    myState = LevelingState.LEVEL;
    if (levelingDataLogger != null) {
      levelingDataLogger.done();
      levelingDataLogger = null;
    }
    driveSubsystem.stopDrive();

    // don't do this: teleop will set it to coast
    // driveSubsystem.setDriveToCoast();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(myState == LevelingState.DONE){
      return true;
    }
    return false;
  }

  @Override
  public LevelingData getLevelingData() {
    return new LevelingData("" + myState, myState.ordinal(), pitch, power);
  }
}
