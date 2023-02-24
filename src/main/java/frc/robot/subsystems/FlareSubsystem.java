package frc.robot.subsystems;

import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.logger.EventLogging.Level;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FlareSubsystem extends SubsystemBase {
  private Spark blinkinMotorController;
  private FlareColor currentFlareColor;

  Logger logger = EventLogging.getLogger(getClass(), Level.INFO);

  public FlareSubsystem() {
    blinkinMotorController = new Spark(2);
    addChild("Blinkin", blinkinMotorController);
    currentFlareColor = FlareColor.YELLOW;
  }

  public void setColor(FlareColor flareColor){
    logger.info ("Set color to {}", flareColor);
    currentFlareColor = flareColor;
  }

  public enum FlareColor{
    PURPLE(0.91), YELLOW(0.69), YELLOWSTROBE(0.15), PURPLESTROBE(0.35);
  
    public final double value;
    private FlareColor(double value){
      this.value = value;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    blinkinMotorController.set(currentFlareColor.value);
  }
}
