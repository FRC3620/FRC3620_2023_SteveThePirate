// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.logger.EventLogging.Level;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CannonSubsystem;

public class CannonClawOutCommand extends CommandBase {
  Logger logger = EventLogging.getLogger(getClass(), Level.INFO);
    
  CannonSubsystem cannonSubsystem;
    double desiredSpeed;
    Timer getStartedTimer;

    public CannonClawOutCommand(CannonSubsystem _subsystem, double _desiredSpeed) {
      // Use addRequirements() here to declare subsystem dependencies.
      //addRequirements(_subsystem);
      cannonSubsystem = _subsystem;
      desiredSpeed = _desiredSpeed;
    } 
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      logger.info("started");
      cannonSubsystem.setClawSpeed(desiredSpeed);
    }
  
    // Called every time the scheduler runs while the command is scheduled.
  
    @Override
    public void execute() 
    {
      cannonSubsystem.setClawSpeed(desiredSpeed);
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      logger.info("ended, interrupted = {}", interrupted);
      cannonSubsystem.setClawSpeed(0);
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
         return false;
    }
  }