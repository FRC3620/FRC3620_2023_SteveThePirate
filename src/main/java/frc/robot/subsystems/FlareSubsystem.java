package frc.robot.subsystems;

import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.logger.EventLogging.Level;
import org.usfirst.frc3620.misc.RobotMode;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FlareSubsystem extends SubsystemBase {
  final int numberOfPixels = 19;
  AddressableLED leds;
  AddressableLEDBuffer ledBuffer;
  FlareColor[] flareColors;
  boolean colorsNeedUpdating = false;
  double onSeconds = 1;
  double offSeconds = 0;
  boolean lightsAreOn = true;
  boolean watchTheClock = true;

  Timer timer = new Timer();
  
  Logger logger = EventLogging.getLogger(getClass(), Level.INFO);

  public FlareSubsystem() {
    this(null, null);
  }

  public FlareSubsystem(String name, Integer port) {
    if (name != null) setName(name);
    if (port == null) port = 9;

    leds = new AddressableLED(port);

    leds.setLength(numberOfPixels);
    ledBuffer = new AddressableLEDBuffer(numberOfPixels);
    flareColors = new FlareColor[numberOfPixels];

    setInterval(1, 0);
    setColor(new FlareColor(new Color(0,0, 0)));

    leds.start();
    timer.start();
  }

  public void setWatchTheClock(boolean watch) {
    watchTheClock = watch;
  }

  public void setInterval(double onSeconds, double offSeconds) {
    this.onSeconds = onSeconds;
    this.offSeconds = offSeconds;
    //SmartDashboard.putNumber("offseconds", offSeconds);
    //SmartDashboard.putNumber("onseconds", onSeconds);
    if (offSeconds == 0) colorsNeedUpdating = true;
  }

  public void setColor(Color color) {
    setColor(new FlareColor(color));
  }

  public void setColor(FlareColor flareColor){
    logger.info ("Set color to {}", flareColor);

    for (var i = 0; i < ledBuffer.getLength(); i++) {
      flareColors[i] = flareColor;
   }
    colorsNeedUpdating = true;
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
      flareColors[i] = flareColor;
    }
    
    colorsNeedUpdating = true;
  }

  RobotMode robotMode = RobotMode.INIT;  

  public void ProcessRobotModeChange(RobotMode robotMode) {
    this.robotMode = robotMode;
    /*
    if (robotMode == RobotMode.TELEOP ) {

      if (DriverStation.getAlliance()==Alliance.Blue) {
        setColor(FlareColor.BLUE);
      }

      if (DriverStation.getAlliance() == Alliance.Red) {
        setColor(FlareColor.RED);
      }

    }
    if (robotMode == RobotMode.AUTONOMOUS ) {

      if (DriverStation.getAlliance()==Alliance.Blue) {
        setColor(FlareColor.BLUE);
      }

      if (DriverStation.getAlliance() == Alliance.Red) {
        setColor(FlareColor.RED);
      }

    }
    */


  }

  public static class FlareColor{
    //PURPLE(0.91), YELLOW(0.69), YELLOWSTROBE(0.15), PURPLESTROBE(0.35);
    public static FlareColor PURPLE = new FlareColor (Color.kPurple);

    public static FlareColor YELLOW = new FlareColor (Color.kYellow);

    public static FlareColor GREEN = new FlareColor(Color.kGreen);

    public static FlareColor RED = new FlareColor(Color.kRed);

    public static FlareColor BLUE = new FlareColor(Color.kBlue);
  
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

    public String toString() {
      return this.value.toString();
    }
  
  }

  @Override
  public void periodic() {
    if (watchTheClock) {
      double matchTime = DriverStation.getMatchTime();

      //SmartDashboard.putNumber("match time", matchTime);
      if (robotMode == RobotMode.TELEOP) {
        if (matchTime > 30) {
          setInterval(1, 0);
        }

        if (matchTime <= 30) {
          setInterval(.75, .75);
        }

        if (matchTime <= 20) {
          setInterval(.3, .3);
        }

        if (matchTime <= 10) {
          setInterval(.2, .2);
        }

        if (matchTime < 5) {
          setInterval(.1, .1);
        }

    }
  }
  
  if (robotMode == RobotMode.DISABLED) {
    setInterval(1, 0);
  }

  if (lightsAreOn) {
    if (offSeconds == 0) {
      lightsAreOn = false; // lie so we turn them on
    } else if (timer.get() > onSeconds) {
      // turn lights off
      for (var i = 0; i < ledBuffer.getLength(); i++) {
        ledBuffer.setRGB(i, 0, 0, 0);
      }
      leds.setData(ledBuffer);
      timer.reset();
      lightsAreOn = false;
    }
  } else if ((offSeconds > 0 && timer.get() > offSeconds) || colorsNeedUpdating) {
      // turn lights on
      for (var i = 0; i < ledBuffer.getLength(); i++) {
        FlareColor flareColor = flareColors[i];
        ledBuffer.setRGB(i, flareColor.getRed(), flareColor.getGreen(), flareColor.getBlue());
      }
      leds.setData(ledBuffer);
      colorsNeedUpdating = false;
      timer.reset();
      lightsAreOn = true;
    }
  }
}
