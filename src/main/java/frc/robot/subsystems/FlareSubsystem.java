package frc.robot.subsystems;

import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.logger.EventLogging.Level;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FlareSubsystem extends SubsystemBase {
  
  AddressableLED leds;
  AddressableLEDBuffer ledBuffer;

  Logger logger = EventLogging.getLogger(getClass(), Level.INFO);

  public FlareSubsystem() {

    leds = new AddressableLED(9);
    leds.setLength(8);;
    ledBuffer = new AddressableLEDBuffer(8);
    setColor(new FlareColor(Color.kBlue));
    leds.start();
  }
  public void setColor(FlareColor flareColor){
    logger.info ("Set color to {}", flareColor);

    for (var i = 0; i < ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      ledBuffer.setRGB(i, flareColor.getRed(), flareColor.getGreen() , flareColor.getBlue());
   }
    leds.setData(ledBuffer);
  }

  public void setColor(FlareColor flareColor, int firstLed, int lastLed){
    logger.info ("Set color to {}", flareColor);

    if (firstLed < 0) {
      firstLed = 0;
    }
    if (lastLed >= ledBuffer.getLength()) {
      lastLed = ledBuffer.getLength() - 1;
    }

    for (var i = firstLed; i <= lastLed; i++) {
      // Sets the specified LED to the RGB values for red
     
      ledBuffer.setRGB(i, flareColor.getRed(), flareColor.getGreen() , flareColor.getBlue());
   }
    leds.setData(ledBuffer);
  }

  public static class FlareColor{
    //PURPLE(0.91), YELLOW(0.69), YELLOWSTROBE(0.15), PURPLESTROBE(0.35);
    public static FlareColor PURPLESTROBE = new FlareColor (Color.kPurple);

    public static FlareColor YELLOWSTROBE = new FlareColor (Color.kYellow);


  
    public final Color value;
    public FlareColor(int R, int G, int B){
      this.value = new Color(R, G, B);
    }
    public FlareColor(Color color){
      this.value = color;
    }

    public int getRed() {
      return (int) Math.round(this.value.red * 255);
    }

    public int getGreen(){
      return (int) Math.round(this.value.green * 255);
    }
    public int getBlue(){
      return (int) Math.round(this.value.blue * 255);
    }
  
  }

 // int m_rainbowFirstPixelHue = 0;

 int m_purpleFirstPixleHue = 0;

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    

    // For every pixel

   
  }
}
