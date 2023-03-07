package frc.robot.commands;

import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.logger.EventLogging.Level;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisionSubsystem.FrontCameraMode;


public class TestPhotonVisionPipelineSwitchCommand extends CommandBase {
  Logger logger = EventLogging.getLogger(getClass(), Level.INFO);
  
  Timer cycleTimer = new Timer();
  Timer delayTimer = new Timer();
  boolean wasCorrect = false;

  VisionSubsystem.FrontCameraMode mode;

  /** Creates a new TestPhotonVisionPipelineSwitchCommand. */
  public TestPhotonVisionPipelineSwitchCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.visionSubsystem.disableCheckTimer();
    cycleTimer.start();
    mode = FrontCameraMode.APRILTAGS;
    setModeAndStartDelayTimer();
  }

  void setModeAndStartDelayTimer() {
    RobotContainer.visionSubsystem.setFrontCameraMode(mode);
    delayTimer.reset();
    wasCorrect = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!wasCorrect) {
      int currentPipeline = RobotContainer.visionSubsystem.getFrontCameraPipelineIndex();
      if (currentPipeline == mode.getPipelineIndex()) {
        logger.info ("switch to {} took {}", mode, delayTimer.get());
        wasCorrect = true;
      }
    }

    if (cycleTimer.advanceIfElapsed(2)) {
      if (!wasCorrect) {
        logger.info ("switch to {} missed", mode);
      }
      if (mode == FrontCameraMode.APRILTAGS) {
        mode = FrontCameraMode.CUBES;
      } else if (mode == FrontCameraMode.CUBES) {
        mode = FrontCameraMode.CONES;
      } else if (mode == FrontCameraMode.CONES) {
        mode = FrontCameraMode.APRILTAGS;
      }
      setModeAndStartDelayTimer();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.visionSubsystem.enableCheckTimer();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
