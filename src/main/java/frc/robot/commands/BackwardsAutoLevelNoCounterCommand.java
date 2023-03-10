// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.logger.IFastDataLogger;
import org.usfirst.frc3620.logger.EventLogging.Level;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ILevelingDataSource;
import frc.robot.LevelingDataLogger;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CannonSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FlareSubsystem.FlareColor;

public class BackwardsAutoLevelNoCounterCommand extends CommandBase implements ILevelingDataSource {
  AHRS ahrs; 
  private DriveSubsystem driveSubsystem;
  private CannonSubsystem cannonSubsystem;

  double power;
  double pitch;
  Timer timer;

  Logger logger = EventLogging.getLogger(getClass(), Level.INFO);

  enum LevelingState {
    LEVEL, TIMED, PRETILTED, TILTED, ARMADJUST, DONE
  }

  LevelingState myState;

  final static boolean doLog = true;
  IFastDataLogger levelingDataLogger = null;
  
  /** Creates a new AutoLevelingCommand. */
  public BackwardsAutoLevelNoCounterCommand(DriveSubsystem driveSubsystem, CannonSubsystem cannonSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSubsystem = driveSubsystem;
    this.cannonSubsystem = cannonSubsystem;
    addRequirements(driveSubsystem);
    myState = LevelingState.LEVEL;
  }

  void setColor(Color color){
    if(RobotContainer.flareSubsystem != null){
      RobotContainer.flareSubsystem.setColor(new FlareColor(color));
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    myState = LevelingState.LEVEL;
    setColor(Color.kPink);
    driveSubsystem.setDriveToBrake();
    driveSubsystem.setTargetHeading(0);


    //cannonSubsystem.setPitch(-117);
    //cannonSubsystem.setElevation(30);
    //cannonSubsystem.setExtension(0);

    if (doLog) {
      levelingDataLogger = LevelingDataLogger.getDataLogger(getClass().getSimpleName(), this);
      levelingDataLogger.start();
    }

    timer = null;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    power = 0;
    pitch = RobotContainer.navigationSubsystem.getPitch();
    double spin = 0;

    if(myState == LevelingState.LEVEL){
      spin = driveSubsystem.getSpinPower();
      //drive
      power = -0.6
      ;
      if(pitch > 13) {
        // we are going uphill, slow down
        myState = LevelingState.TIMED;
        logger.info("switching to {}, pitch = {}", myState, pitch);
        setColor(Color.kBlue);
      }
    }

    if(myState == LevelingState.TIMED){
      if(timer == null){
        timer = new Timer();
      }
      timer.start();
      power = -0.4;
      if(timer.advanceIfElapsed(.5)){
        myState = LevelingState.PRETILTED;
        logger.info("switching to {}, pitch = {}", myState, pitch);
        setColor(Color.kYellow);
        timer = null;
      }
    }

    if(myState == LevelingState.PRETILTED){
      power = -0.1;
      if(timer == null){
        timer = new Timer();
        timer.start();
      }
      if(timer.advanceIfElapsed(.5)){
        myState = LevelingState.TILTED;
        logger.info("switching to {}, pitch = {}", myState, pitch);
        setColor(Color.kPurple);
        timer = null;
      }
    }
    
    if(myState == LevelingState.TILTED){
      power = -0.1;
      if(pitch < 12.5){ //was -10
        // we are still going up hill, but not as much. it must be swinging?
        myState = LevelingState.DONE;
        logger.info("switching to {}, pitch = {}", myState, pitch);
        power = 0;
        setColor(Color.kGreen);
      }
    }

    if(myState == LevelingState.ARMADJUST){
      if(pitch > 10){
        cannonSubsystem.setElevation(30);
      }
      myState = LevelingState.DONE;
    }

    /*if(myState == LevelingState.COUNTER){
      power = -.2;
      if(pitch > 3){ //was 10
        power = 0;
        logger.info("switching to done, pitch = {}", pitch);
        myState = LevelingState.DONE;
        setColor(Color.kGreen);
      }
    }*/

    if (power == 0) {
      driveSubsystem.stopDrive();
    } else {
      driveSubsystem.autoDrive(0, power, spin);
    }
  

    if (levelingDataLogger != null) levelingDataLogger.update();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
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
    if(myState == LevelingState.DONE) {
      driveSubsystem.xMode();
      if(timer == null){
        timer = new Timer();
        timer.start();
      }
      if(timer.advanceIfElapsed(2)){
        timer = null;
        return true;
      }
    }
    return false;
  }

  @Override
  public LevelingData getLevelingData() {
    return new LevelingData("" + myState, myState.ordinal(), pitch, power);
  }
}
