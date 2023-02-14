package frc.robot.commands;

import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.logger.EventLogging.Level;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.VisionSubsystem.FrontCameraMode;

public class SeeCubeCommand extends CommandBase {
  Logger logger = EventLogging.getLogger(getClass(), Level.INFO);

  FrontCameraMode savedCameraMode;

  public SeeCubeCommand() {
    addRequirements(RobotContainer.visionSubsystem);
  }

  @Override
  public void initialize() {
    savedCameraMode = RobotContainer.visionSubsystem.setFrontCameraMode(FrontCameraMode.CUBES);
    logger.info ("camera mode was {}", savedCameraMode);
  }

  @Override
  public void end(boolean interrupted) {
    logger.info ("setting camera mode back to {}", savedCameraMode);
    RobotContainer.visionSubsystem.setFrontCameraMode(savedCameraMode);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
