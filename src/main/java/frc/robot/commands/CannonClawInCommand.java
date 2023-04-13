// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.logger.EventLogging.Level;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CannonSubsystem;

public class CannonClawInCommand extends CommandBase {
    Logger logger = EventLogging.getLogger(getClass(), Level.INFO);
    /** Creates a new CannonExtendCommand. */
    CannonSubsystem cannonSubsystem;
    double desiredSpeed;
    Timer getStartedTimer;
    /**
     * 
     * Creates a new MoveTurretCommand.
     */
    public CannonClawInCommand(CannonSubsystem _subsystem, double _desiredSpeed) {
      // Use addRequirements() here to declare subsystem dependencies.
      //addRequirements(_subsystem);
      cannonSubsystem = _subsystem;
      desiredSpeed = _desiredSpeed;
    } 
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      logger.info("started");
      cannonSubsystem.setClawPower(desiredSpeed);
      cannonSubsystem.setHasPiece(false);
    }
  
    // Called every time the scheduler runs while the command is scheduled.
  
    @Override
    public void execute() {
      cannonSubsystem.setClawPower(desiredSpeed);
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      logger.info ("done, interrupted = {}", interrupted);
      cannonSubsystem.setClawPower(.1);
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      if (getStartedTimer == null) {
        getStartedTimer = new Timer();
        getStartedTimer.reset();
        getStartedTimer.start();
      } else {
        if (getStartedTimer.get() > 0.5) {
          if (Math.abs(cannonSubsystem.getClawPower()) < 500) {
            cannonSubsystem.setHasPiece(true);
            getStartedTimer = null;
            RobotContainer.flareSubsystem.setColor(Color.kGreen);
            return true;
            } else {
              return false;
            }
          }
        }
         return false;
    }
  }