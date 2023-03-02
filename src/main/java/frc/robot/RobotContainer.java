package frc.robot;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.commands.*;
import frc.robot.commands.DriveToAprilTagCommand.Position;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.FlareSubsystem.FlareColor;
import frc.robot.subsystems.VisionSubsystem.FrontCameraMode;

import java.util.Set;

import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.logger.LogCommand;
import org.usfirst.frc3620.logger.EventLogging.Level;
import org.usfirst.frc3620.misc.CANDeviceFinder;

import frc.robot.subsystems.CannonSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FlareSubsystem;
import frc.robot.subsystems.INavigationSubsystem;
import frc.robot.subsystems.NavXNavigationSubsystem;
import frc.robot.subsystems.OdometrySubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import org.usfirst.frc3620.misc.CANDeviceType;
import org.usfirst.frc3620.misc.DPad;
import org.usfirst.frc3620.misc.JoystickAnalogButton;
import org.usfirst.frc3620.misc.RobotParametersContainer;
import org.usfirst.frc3620.misc.XBoxConstants;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  public final static Logger logger = EventLogging.getLogger(RobotContainer.class, Level.INFO);
  
  // need this
  public static CANDeviceFinder canDeviceFinder;
  public static RobotParameters robotParameters;

  // hardware here...
  private static DigitalInput practiceBotJumper;

  public static PneumaticsModuleType pneumaticModuleType = null;

  // subsystems here
  public static DriveSubsystem driveSubsystem;
  public static INavigationSubsystem navigationSubsystem;
  public static VisionSubsystem visionSubsystem;
  public static OdometrySubsystem odometrySubsystem;
  public static CannonSubsystem cannonSubsystem;
  public static FlareSubsystem flareSubsystem, balanceLights;

  // joysticks here....
  public static Joystick driverJoystick;
  public static Joystick operatorJoystick;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    canDeviceFinder = new CANDeviceFinder();

    robotParameters = RobotParametersContainer.getRobotParameters(RobotParameters.class);
    logger.info ("got parameters for chassis '{}'", robotParameters.getName());

    practiceBotJumper = new DigitalInput(0);
    SendableRegistry.add(practiceBotJumper, "RobotContainer", "Practice Bot Jumper");
    boolean iAmACompetitionRobot = amIACompBot();
    if (!iAmACompetitionRobot) {
      logger.warn("this is a test chassis, will try to deal with missing hardware!");
    }

    if (canDeviceFinder.isDevicePresent(CANDeviceType.REV_PH, 1, "REV PH") || iAmACompetitionRobot) {
      pneumaticModuleType = PneumaticsModuleType.REVPH;
    } else if (canDeviceFinder.isDevicePresent(CANDeviceType.CTRE_PCM, 0, "CTRE PCM")) {
      pneumaticModuleType = PneumaticsModuleType.CTREPCM;
    }
   
    makeSubsystems();

    // CAN bus ok?
    Set<CANDeviceFinder.NamedCANDevice> missingDevices = canDeviceFinder.getMissingDeviceSet();
    if (missingDevices.size() > 0) {
      SmartDashboard.putBoolean("can.ok", false);
      SmartDashboard.putString("can.missing", missingDevices.toString());
    } else {
      SmartDashboard.putBoolean("can.ok", true);
      SmartDashboard.putString("can.missing", "");
    }

    // Configure the button bindings
    configureButtonBindings();

    setupSmartDashboardCommands();

    setupAutonomousCommands();
  }

  private void makeSubsystems() {
    navigationSubsystem = new NavXNavigationSubsystem();
    driveSubsystem = new DriveSubsystem(navigationSubsystem);
    visionSubsystem = new VisionSubsystem();
    odometrySubsystem = new OdometrySubsystem(navigationSubsystem, DriverStation.getAlliance(), robotParameters.swerveParameters, driveSubsystem);
    cannonSubsystem = new CannonSubsystem();
    balanceLights = new FlareSubsystem("balanceLights", 8);
    balanceLights.setWatchTheClock(false);
    //flareSubsystem = new FlareSubsystem();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    driverJoystick = new Joystick(0);
    operatorJoystick = new Joystick(1);

    DPad driverDPad = new DPad(driverJoystick, 0);
    DPad operatorDPad = new DPad(operatorJoystick, 0);

    //Driver
    new JoystickButton(driverJoystick, XBoxConstants.BUTTON_A)
            .whileTrue(new XModeCommand(driveSubsystem));

    new JoystickButton(driverJoystick, XBoxConstants.BUTTON_X)
            .onTrue(new ResetNavXCommand());
    
    new JoystickButton(driverJoystick, XBoxConstants.BUTTON_Y)
            .onTrue(new SetNavX180Command());
            
    new JoystickAnalogButton(driverJoystick, XBoxConstants.AXIS_LEFT_TRIGGER)
            .onTrue(new CannonClawInCommand(cannonSubsystem, 0.6));

    new JoystickAnalogButton(driverJoystick, XBoxConstants.AXIS_RIGHT_TRIGGER)
            .whileTrue(new CannonClawOutCommand(cannonSubsystem, -0.8));

    // operator colors
    if (flareSubsystem != null) {
      new JoystickButton(operatorJoystick, XBoxConstants.BUTTON_BACK)
              .onTrue(new InstantCommand (() -> flareSubsystem.setColor(FlareColor.PURPLE)));

      new JoystickButton(operatorJoystick, XBoxConstants.BUTTON_START)
              .onTrue(new InstantCommand (() -> flareSubsystem.setColor(FlareColor.YELLOW)));
    }

    // operator cannon stuff
    new JoystickButton(operatorJoystick, XBoxConstants.BUTTON_RIGHT_BUMPER)
            .onTrue(new SetCannonLocationCommand(CannonLocation.coneHighLocation));

    new JoystickAnalogButton(operatorJoystick, XBoxConstants.AXIS_RIGHT_TRIGGER)
            .onTrue(new SetCannonLocationCommand(CannonLocation.coneMidLocation));

    new JoystickButton(operatorJoystick, XBoxConstants.BUTTON_LEFT_BUMPER)
            .onTrue(new SetCannonLocationCommand(CannonLocation.cubeHighLocation));

    new JoystickAnalogButton(operatorJoystick, XBoxConstants.AXIS_LEFT_TRIGGER)
            .onTrue(new SetCannonLocationCommand(CannonLocation.cubeMidLocation));

    new JoystickButton(operatorJoystick, XBoxConstants.BUTTON_X)
            .onTrue(new SetCannonLocationCommand(CannonLocation.sidewaysConeLocation));

    new JoystickButton(operatorJoystick, XBoxConstants.BUTTON_B)
            .onTrue(new SetCannonLocationCommand(CannonLocation.chuteLocation));
            
    new JoystickButton(operatorJoystick, XBoxConstants.BUTTON_A)
            .onTrue(new SetCannonLocationCommand(CannonLocation.coneFloorPickLocation));

    new JoystickButton(operatorJoystick, XBoxConstants.BUTTON_Y)
            .onTrue(new SetCannonLocationCommand(CannonLocation.stationLocation));
             
    new JoystickAnalogButton(operatorJoystick, XBoxConstants.AXIS_RIGHT_Y, 0.2)
            .whileTrue(new CannonElevatePowerCommand(cannonSubsystem, 8));

    new JoystickAnalogButton(operatorJoystick, XBoxConstants.AXIS_RIGHT_Y, -0.2)
            .whileTrue(new CannonElevatePowerCommand(cannonSubsystem, -8));

    new JoystickAnalogButton(operatorJoystick, XBoxConstants.AXIS_LEFT_Y, 0.2)
            .whileTrue(new CannonExtendPowerCommand(cannonSubsystem, 4));

    new JoystickAnalogButton(operatorJoystick, XBoxConstants.AXIS_LEFT_Y, -0.2)
            .whileTrue(new CannonExtendPowerCommand(cannonSubsystem, -4));   
             
    operatorDPad.up().onTrue(new SetCannonLocationCommand(CannonLocation.parkLocation));
    operatorDPad.left().whileTrue(new CannonPitchPowerCommand(cannonSubsystem, 5));
    operatorDPad.right().whileTrue(new CannonPitchPowerCommand(cannonSubsystem, -5));
  }

  /*public static double getOperatorJoystickRightY() {
    double axisValue = operatorJoystick.getRawAxis(XBoxConstants.AXIS_RIGHT_Y); //Grabs the joystick value
    if (axisValue < 0.2 && axisValue > -0.2) { //Since the joystick doesnt stay at zero, make it not give a false value
      return 0;
    }
    return -axisValue;
  }

  public static double getOperatorJoystickLeftY() {
    double axisValue = operatorJoystick.getRawAxis(XBoxConstants.AXIS_LEFT_Y); //Grabs the joystick value
    if (axisValue < 0.2 && axisValue > -0.2) { //Since the joystick doesnt stay at zero, make it not give a false value
      return 0;
    }
    return -axisValue;
  }*/

  private void setupSmartDashboardCommands() {
    // DriveSubsystem
    SmartDashboard.putData("Strafe to target", new StrafeToAprilTagCommand(driveSubsystem, visionSubsystem));
    SmartDashboard.putData("Move to target", new LocateAprilTagCommand(driveSubsystem, visionSubsystem));
    SmartDashboard.putData("Drive to apirl tag", new DriveToAprilTagCommand(3, Position.MIDDLE, driveSubsystem, visionSubsystem, odometrySubsystem));
    SmartDashboard.putData("Updated Move to April Tag", new UpdatedLocateAprilTagCommand(driveSubsystem, visionSubsystem));
    SmartDashboard.putData("AprilTagAutoTestCommand", new AprilTagAutoTestCommand(driveSubsystem, visionSubsystem));
    SmartDashboard.putData("Drive to coordinate", new DriveToCoordinateCommand(FieldLocation.humanStart, 0.2, 0.1, 180, driveSubsystem));
    SmartDashboard.putData("Test Coordinate Auto", new TestCoordinateAuto(driveSubsystem));
    SmartDashboard.putData("Longer Test Coordinate Auto", new LongerTestCoordinateAuto(driveSubsystem));
    SmartDashboard.putData("TurnToGamePieceCommand", new TurnToGamePieceCommand(driveSubsystem, visionSubsystem));
    SmartDashboard.putData("Drive to Cone", new DriveToGamePieceCommand(FrontCameraMode.CONES, driveSubsystem, visionSubsystem, cannonSubsystem));
    SmartDashboard.putData("Drive to Cube", new DriveToGamePieceCommand(FrontCameraMode.CUBES, driveSubsystem, visionSubsystem, cannonSubsystem));
    SmartDashboard.putData("Simple test auto", new SimpleTestAuto(driveSubsystem));
    SmartDashboard.putData("Auto Leveling Command", new AutoLevelingCommand(driveSubsystem, cannonSubsystem));
    SmartDashboard.putData("Backwards Auto Leveling Command", new BackwardsAutoLevelingCommand(driveSubsystem, cannonSubsystem));
    SmartDashboard.putData("RunWheelsForwardButton", new RunWheelsForwardButton());
    SmartDashboard.putData("RotateWheelsButton", new RotateWheelsButton());
    SmartDashboard.putData("Auto Level No Counter", new AutoLevelNoCounterCommand(driveSubsystem, cannonSubsystem));

    // Autos
    SmartDashboard.putData("Mid1BalanceAuto", new Mid1BalanceAuto(driveSubsystem, visionSubsystem, cannonSubsystem, odometrySubsystem));
    SmartDashboard.putData("Human1BalanceAuto", new Human1BalanceAuto(driveSubsystem, visionSubsystem, cannonSubsystem, odometrySubsystem));
    SmartDashboard.putData("Wall1BalanceAuto", new Wall1BalanceAuto(driveSubsystem, visionSubsystem, cannonSubsystem, odometrySubsystem));
    SmartDashboard.putData("Human2NoBalanceAuto", new Human2NoBalanceAuto(driveSubsystem, visionSubsystem, cannonSubsystem, odometrySubsystem));

    // Cannon
    SmartDashboard.putData("ExtendCommand0" , new CannonExtendCommand(cannonSubsystem, 0));
    SmartDashboard.putData("ExtendCommand5" , new CannonExtendCommand(cannonSubsystem, 5));
    SmartDashboard.putData("ExtendCommand10" , new CannonExtendCommand(cannonSubsystem, 10));
    SmartDashboard.putData("ExtendCommand20" , new CannonExtendCommand(cannonSubsystem, 20));
    SmartDashboard.putData("Turn off extension", new InstantCommand(() -> cannonSubsystem.disableExtension()));
    SmartDashboard.putData("ElevateHorizontal", new CannonElevateCommand(cannonSubsystem, 0));
    SmartDashboard.putData("ElevateCommand30", new CannonElevateCommand(cannonSubsystem, 30));
    SmartDashboard.putData("ElevateCommand60", new CannonElevateCommand(cannonSubsystem, 60));
    SmartDashboard.putData("ElevateHome", new CannonElevateCommand(cannonSubsystem, 90));
    SmartDashboard.putData("PitchCommand-10", new CannonPitchCommand(cannonSubsystem, -10));
    SmartDashboard.putData("PitchCommand-70", new CannonPitchCommand(cannonSubsystem, -70));
    SmartDashboard.putData("PitchCommand-90", new CannonPitchCommand(cannonSubsystem, -90));

    SmartDashboard.putData("ClawIn", new CannonClawInCommand(cannonSubsystem, 0.2));
    SmartDashboard.putData("ClawOut", new CannonClawInCommand(cannonSubsystem, -0.2));
    SmartDashboard.putData("ClawStop", new CannonClawInCommand(cannonSubsystem, 0));
    SmartDashboard.putData("MidLocation", new SetCannonLocationCommand(CannonLocation.coneMidLocation));
    SmartDashboard.putData("HighLocation", new SetCannonLocationCommand(CannonLocation.coneHighLocation));
    SmartDashboard.putData("ParkLocation", new SetCannonLocationCommand(CannonLocation.parkLocation));

    // Odometry and Vision Tests
    SmartDashboard.putData(new InstrumentOdometryAndVisionCommand());
    SmartDashboard.putData(new SeeConeCommand());
    SmartDashboard.putData(new SeeCubeCommand());
    SmartDashboard.putData(new TestPhotonVisionPipelineSwitchCommand());

    // Autonomous

  }

  SendableChooser<Command> chooser = new SendableChooser<>();
  public void setupAutonomousCommands() {
    SmartDashboard.putData("Auto mode", chooser);
    chooser.setDefaultOption("Do nothing", new LogCommand("no autonomous specified, did nothing"));
    chooser.addOption("April Tag Auto Test", new AprilTagAutoTestCommand(driveSubsystem, visionSubsystem));
    chooser.addOption("Mid1BalanceAuto", new Mid1BalanceAuto(driveSubsystem, visionSubsystem, cannonSubsystem, odometrySubsystem));
    chooser.addOption("Human1BalanceAuto", new Human1BalanceAuto(driveSubsystem, visionSubsystem, cannonSubsystem, odometrySubsystem));
    chooser.addOption("Wall1BalanceAuto", new Wall1BalanceAuto(driveSubsystem, visionSubsystem, cannonSubsystem, odometrySubsystem));
  }

  static double driverStrafeDeadzone = 0.1;

  public static double getDriveVerticalJoystick() {
    double axisValue = driverJoystick.getRawAxis(XBoxConstants.AXIS_LEFT_Y);
    SmartDashboard.putNumber("driver.y.raw", axisValue);
    if (Math.abs(axisValue) < driverStrafeDeadzone) {
      return 0;
    }
    if (axisValue < 0){
      return (axisValue*axisValue);
    }
    return -axisValue*axisValue;
  }

  public static double getDriveHorizontalJoystick() {
    double axisValue = driverJoystick.getRawAxis(XBoxConstants.AXIS_LEFT_X);
    SmartDashboard.putNumber("driver.x.raw", axisValue);
    if (Math.abs(axisValue) < driverStrafeDeadzone) {
      return 0;
    }
    if (axisValue < 0){
      return -(axisValue*axisValue);
    }
    return axisValue*axisValue;
  }

  static double driverSpinDeadzone = 0.1;
  public static double getDriveSpinJoystick() {
    double axisValue = driverJoystick.getRawAxis(XBoxConstants.AXIS_RIGHT_X);
    SmartDashboard.putNumber("driver.spin.raw", axisValue);
    double rv = 0;
    if (Math.abs(axisValue) >= driverSpinDeadzone) {
      rv = axisValue*axisValue;
      if (axisValue < 0){
        rv = -rv;
      }
    }
    SmartDashboard.putNumber("driver.spin.processed", rv);
    return rv;
  }

  /*static double operatorDeadzone = 0.2;

  public static double getOperatorLeftY() {
    double axisValue = operatorJoystick.getRawAxis(XBoxConstants.AXIS_LEFT_Y);
    if (Math.abs(axisValue) < operatorDeadzone) {
      return 0;
    }
    if (axisValue < 0){
      return -(axisValue*axisValue);
    }
    return axisValue*axisValue;
  }*/

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    //return new GoldenAutoCommand(driveSubsystem, shooterSubsystem, VisionSubsystem, intakeSubsystem);
    return chooser.getSelected();
  }

  /**
   * Determine if this robot is a competition robot.
   *
   * It is if it's connected to an FMS.
   *
   * It is if it is missing a grounding jumper on DigitalInput 0.
   *
   * It is if the robot_parameters.json says so for this MAC address.
   *
   * @return true if this robot is a competition robot.
   */
  public static boolean amIACompBot() {
    if (DriverStation.isFMSAttached()) {
      return true;
    }

    if(practiceBotJumper.get() == true){
      return true;
    }

    if (robotParameters.isCompetitionRobot()) {
      return true;
    }

    return false;
  }

  /**
   * Determine if we should make software objects, even if the device does 
   * not appear on the CAN bus.
   *
   * We should if it's connected to an FMS.
   *
   * We should if it is missing a grounding jumper on DigitalInput 0.
   *
   * We should if the robot_parameters.json says so for this MAC address.
   * 
   * @return true if we should make all software objects for CAN devices
   */
  public static boolean shouldMakeAllCANDevices() {
    if (DriverStation.isFMSAttached()) {
      return true;
    }

    if (!Robot.isReal()) {
      return false;
    }

    if(practiceBotJumper.get() == true){
      return true;
    }

    if (robotParameters.shouldMakeAllCANDevices()) {
      return true;
    }

    return false;
  }

  

}