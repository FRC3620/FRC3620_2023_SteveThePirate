// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FlareSubsystem;
import frc.robot.subsystems.FlareSubsystem.FlareColor;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** An example command that uses an example subsystem. */
public class RotateWheelsButton extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
  private final DriveSubsystem driveSubsystem;
  private final FlareSubsystem flareSubsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param DriveSubsystem The subsystem used by this command.
   */
  public RotateWheelsButton() {
    driveSubsystem = RobotContainer.driveSubsystem;
    flareSubsystem = RobotContainer.flareSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("DiagnosticsAzimuthMotorTest", false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSubsystem.testDrive(.25, 0);
    SmartDashboard.putBoolean("DiagnosticsAzimuthMotorTest", areAllAzimuthsok());
   
    if(this.areAllAzimuthsok()){
      flareSubsystem.setColor(FlareColor.GREENSTROBE, 0, 1);
    
    } else {
      flareSubsystem.setColor(FlareColor.REDSTROBE, 0, 1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.testDrive(0, 0);

  }

  public boolean areTheyClose(double a, double b) {

    double larger = Double.max(a, b);

    double absoluteValue = Math.abs(a - b);

    if (absoluteValue / larger > 0.1) {
      return false;
    }
    return true;
  }

  public boolean areAllAzimuthsok(){
    double lf = driveSubsystem.leftFrontAzimuthEncoder.getVelocity();
    double lb = driveSubsystem.leftBackAzimuthEncoder.getVelocity();
    double rf = driveSubsystem.rightFrontAzimuthEncoder.getVelocity();
    double rb = driveSubsystem.rightBackAzimuthEncoder.getVelocity();

    if (! areTheyClose(lf, rf)) return false;
    if (! areTheyClose(lf, rb)) return false;
    if (! areTheyClose(lf, lb)) return false;
    if (! areTheyClose(lb, rb)) return false;
    if (! areTheyClose(lb, rf)) return false;
    if (! areTheyClose(rb, rf)) return false;
  
    return true;
  }

  

}

// Returns true when the command should end.
