package frc.robot.subsystems;

import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.logger.EventLogging.Level;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FlareSubsystem extends SubsystemBase {
  private Spark blinkinMotorController;
  private FlareColor currentFlareColor;
  AddressableLED leds;
  AddressableLEDBuffer ledBuffer;

  Logger logger = EventLogging.getLogger(getClass(), Level.INFO);

  Timer timer = new Timer();

  public FlareSubsystem() {
    blinkinMotorController = new Spark(2);
    addChild("Blinkin", blinkinMotorController);
    currentFlareColor = FlareColor.YELLOW;

    leds = new AddressableLED(9);
    leds.setLength(8);;
    ledBuffer = new AddressableLEDBuffer(8);

    timer.start();


    for (var i = 0; i < ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      ledBuffer.setRGB(i, 255, 0, 0);
   }
    leds.setData(ledBuffer);
    leds.start();
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

 // int m_rainbowFirstPixelHue = 0;

 int m_purpleFirstPixleHue = 0;

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    blinkinMotorController.set(currentFlareColor.value);

    // For every pixel

    for (var i = 0; i < ledBuffer.getLength();i++) {

      // Calculate the hue - hue is easier for rainbows because the color

      // shape is a circle so only one value needs to precess
    
        var hue = (m_purpleFirstPixleHue + (i * 95 / ledBuffer.getLength())) % 91;

    //  var hue = (m_rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;

      // Set the value

    //  ledBuffer.setHSV(i, hue, 255, 128);

    ledBuffer.setHSV(i, 298, 100, 40);

    }
   // leds.setData(ledBuffer);
    leds.setData(ledBuffer);

    // Increase by to make the rainbow "move"

   // m_rainbowFirstPixelHue += 3;
   

    // Check bounds

   // m_rainbowFirstPixelHue %= 180;
   m_purpleFirstPixleHue %= 91
   ;
  }
}
