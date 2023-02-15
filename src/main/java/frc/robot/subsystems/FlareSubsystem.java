// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle.LEDStripType;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class FlareSubsystem extends SubsystemBase {
  /** Creates a new FlareSubsystem. */
  private Spark Flare;
  private double CurrentFlareColor;

  public FlareSubsystem() {
    Flare = new Spark(2);
    CurrentFlareColor = FlareColor.PURPLE.value;
    CurrentFlareColor = FlareColor.YELLOW.value;
  }

  public void setColor(FlareColor flareColor){
    CurrentFlareColor = flareColor.value;
  }

  public enum FlareColor{
    PURPLE(0.91), YELLOW(0.69);
  
    public final double value;
    private FlareColor(double value){
      this.value = value;
    }
  }




  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    Flare.set(CurrentFlareColor);
  }



}

