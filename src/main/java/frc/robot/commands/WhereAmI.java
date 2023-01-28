// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Observer;
import java.util.Optional;

import edu.wpi.first.math.estimator.KalmanFilterLatencyCompensator.ObserverSnapshot;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.VisionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class WhereAmI extends CommandBase {
  public WhereAmI() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  @Override
  public void execute() {
    VisionSubsystem.AllAprilTagsInPicture allAprilTagsInPicture = VisionSubsystem.allAprilTagsInPicture;
    if(allAprilTagsInPicture != null)
    {
      Integer idOfClosetTag = allAprilTagsInPicture.getIdOfClosestTag();
      if(idOfClosetTag != null)
      {
        Transform3d transform3d = allAprilTagsInPicture.getTransform3d(idOfClosetTag);
        SmartDashboard.putNumber("whereami.closest tag id", idOfClosetTag);
        SmartDashboard.putNumber("whereami.closest tag distance x", transform3d.getX());
        SmartDashboard.putNumber("whereami.closest tag distance y", transform3d.getY());
        SmartDashboard.putNumber("whereami.closest tag distance z", transform3d.getZ());

        VisionSubsystem.foo(idOfClosetTag);
      }

    }
  }

  @Override
  public boolean runsWhenDisabled(){
    return true;
  }
}
