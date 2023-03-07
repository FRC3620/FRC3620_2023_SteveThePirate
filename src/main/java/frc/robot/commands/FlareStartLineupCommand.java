// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FlareSubsystem;
import frc.robot.subsystems.VisionSubsystem;

//DoubleTopic dblYaw = inst.getDoubleTopic("/photon/Front Camera 2/targetYaw");

public class FlareStartLineupCommand extends CommandBase {
  FlareSubsystem flareSubsystem;
  //final DoubleSubscriber dblPhotonSubscriber;

  
  /** Creates a new FlareStartLineupCommand. */
  //public FlareStartLineupCommand(DoubleYaw dblYaw) {

  // dblPhotonSubscriber = dblYaw.subscribe(0.0);
   
   


    // Use addRequirements() here to declare subsystem dependencies.
 // }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

   



   
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
