// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class RunWheelsForwardButton extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem driveSubsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param DriveSubsystem The subsystem used by this command.
   */
  public  RunWheelsForwardButton() {
    driveSubsystem = RobotContainer.driveSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSubsystem.testDrive(0, .25);


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.testDrive(0, 0);

  }

  // Returns true when the command should end.

  //boolean false= 1 or more motor bad

    public boolean areTheyClose(double a, double b){
  
      double larger = Double.max(a, b);
  
      double absoluteValue = Math.abs(a-b);
    
      if (absoluteValue/larger>0.1){return false;}
    return true;
    }
    public boolean areAllwheelsok(){
      double lf = driveSubsystem.leftFrontDriveEncoder.getVelocity();
      double lb = driveSubsystem.leftBackDriveEncoder.getVelocity();
      double rf = driveSubsystem.rightFrontDriveEncoder.getVelocity();
      double rb = driveSubsystem.rightBackDriveEncoder.getVelocity();
  
      if (! areTheyClose(lf, rf)) return false;
      if (! areTheyClose(lf, rb)) return false;
      if (! areTheyClose(lf, lb)) return false;
      if (! areTheyClose(lb, rb)) return false;
      if (! areTheyClose(lb, rf)) return false;
      if (! areTheyClose(rb, rf)) return false;
    return true;
    }
  

  }