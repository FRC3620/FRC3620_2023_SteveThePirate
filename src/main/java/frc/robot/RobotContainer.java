package frc.robot;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.commands.*;
import frc.robot.commands.DriveToAprilTagCommand.Position;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.FlareSubsystem.FlareColor;
import frc.robot.subsystems.VisionSubsystem.FrontCameraMode;

import java.util.Set;
import java.util.function.Supplier;

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
import org.usfirst.frc3620.misc.ChameleonController;
import org.usfirst.frc3620.misc.DPad;
import org.usfirst.frc3620.misc.JoystickAnalogButton;
import org.usfirst.frc3620.misc.RobotMode;
import org.usfirst.frc3620.misc.RobotParametersContainer;
import org.usfirst.frc3620.misc.XBoxConstants;
import org.usfirst.frc3620.misc.ChameleonController.ControllerType;
import org.usfirst.frc3620.misc.FlySkyConstants;

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
  public static FlareSubsystem flareSubsystem;

  // joysticks here....
  public static Joystick rawDriverJoystick;
  public static ChameleonController driverJoystick;
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

    /*
    if (canDeviceFinder.isDevicePresent(CANDeviceType.REV_PH, 1, "REV PH") || iAmACompetitionRobot) {
      pneumaticModuleType = PneumaticsModuleType.REVPH;
    } else if (canDeviceFinder.isDevicePresent(CANDeviceType.CTRE_PCM, 0, "CTRE PCM")) {
      pneumaticModuleType = PneumaticsModuleType.CTREPCM;
    }
    */
   
    makeSubsystems();

    // CAN bus ok?
    Set<CANDeviceFinder.NamedCANDevice> missingDevices = canDeviceFinder.getMissingDeviceSet();
    if (missingDevices.size() > 0) {
      SmartDashboard.putBoolean("can.ok", false);
      SmartDashboard.putString("can.missing", missingDevices.toString());
      flareSubsystem.setColor(Color.kRed);
    } else {
      SmartDashboard.putBoolean("can.ok", true);
      SmartDashboard.putString("can.missing", "");
      flareSubsystem.setColor(Color.kLightGreen);
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
    flareSubsystem = new FlareSubsystem();
  }

  public String getDriverControllerName() {
    return rawDriverJoystick.getName();
  }

  public void setDriverControllerName(ControllerType driveControllerType) {
    driverJoystick.setCurrentControllerType(driveControllerType);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    rawDriverJoystick = new Joystick(0);
    driverJoystick = new ChameleonController(rawDriverJoystick);
    operatorJoystick = new Joystick(1);

    DPad operatorDPad = new DPad(operatorJoystick, 0);

    //Driver
    driverJoystick.button(XBoxConstants.BUTTON_A, FlySkyConstants.BUTTON_SWD)
            .whileTrue(new XModeCommand(driveSubsystem));

    driverJoystick.button(XBoxConstants.BUTTON_X, FlySkyConstants.BUTTON_SWC)
            .onTrue(new ResetNavXCommand());
    
    driverJoystick.button(XBoxConstants.BUTTON_Y, FlySkyConstants.BUTTON_SWA)
            .onTrue(new SetNavX180Command());
            
    driverJoystick.analogButton(XBoxConstants.AXIS_LEFT_TRIGGER, FlySkyConstants.AXIS_SWE)
            .onTrue(new CannonClawInCommand(cannonSubsystem, 0.8)); // was 0.6

    driverJoystick.analogButton(XBoxConstants.AXIS_RIGHT_TRIGGER, FlySkyConstants.AXIS_SWH)
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
            .onTrue(new SetCannonLocationCommand(CannonLocation.cubePickLocation));

    new JoystickButton(operatorJoystick, XBoxConstants.BUTTON_B)
            .onTrue(new SetCannonLocationCommand(CannonLocation.chuteLocation));
            
    new JoystickButton(operatorJoystick, XBoxConstants.BUTTON_A)
            .onTrue(new SetCannonLocationCommand(CannonLocation.coneFloorPickLocation));

    new JoystickButton(operatorJoystick, XBoxConstants.BUTTON_Y)
            .onTrue(new SetCannonLocationCommand(CannonLocation.stationLocation));
             
            
    // positive y-axis is when you pull it down
    // DISABLE THIS FOR COMPETITION!
    new JoystickAnalogButton(operatorJoystick, XBoxConstants.AXIS_RIGHT_Y, 0.2)
            .whileTrue(new CannonElevatePowerCommand(cannonSubsystem, -24));

    // negative y-axis is when you pull it up
    // YOU CAN LEAVE THIS ENABLED FOR COMPETITION.
    new JoystickAnalogButton(operatorJoystick, XBoxConstants.AXIS_RIGHT_Y, -0.2)
            .whileTrue(new CannonElevatePowerCommand(cannonSubsystem, 24));

    // positive y-axis is when you pull it down
    new JoystickAnalogButton(operatorJoystick, XBoxConstants.AXIS_LEFT_Y, 0.2)
            .whileTrue(new CannonExtendPowerCommand(cannonSubsystem, -4));

    // negative y-axis is when you pull it up
    new JoystickAnalogButton(operatorJoystick, XBoxConstants.AXIS_LEFT_Y, -0.2)
            .whileTrue(new CannonExtendPowerCommand(cannonSubsystem, 4));   
             
    operatorDPad.up().onTrue(new SetCannonLocationCommand(CannonLocation.parkLocation));
    operatorDPad.left().whileTrue(new CannonPitchPowerCommand(cannonSubsystem, 20));
    operatorDPad.right().whileTrue(new CannonPitchPowerCommand(cannonSubsystem, -20));
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
    SmartDashboard.putData("Drive to coordinate", new DriveToCoordinateCommand(FieldLocation.humanGrabSecondPiece, 0.2, 0.1, 135, driveSubsystem));
    SmartDashboard.putData("Test Coordinate Auto", new TestCoordinateAuto(driveSubsystem));
    SmartDashboard.putData("Longer Test Coordinate Auto", new LongerTestCoordinateAuto(driveSubsystem));
    SmartDashboard.putData("TurnToGamePieceCommand", new TurnToGamePieceCommand(driveSubsystem, visionSubsystem));
    SmartDashboard.putData("Drive to Cone", new DriveToGamePieceCommand(FrontCameraMode.CONES, driveSubsystem, visionSubsystem, cannonSubsystem));
    SmartDashboard.putData("Drive to Cube", new DriveToGamePieceCommand(FrontCameraMode.CUBES, driveSubsystem, visionSubsystem, cannonSubsystem));
    SmartDashboard.putData("Simple test auto", new SimpleTestAuto(driveSubsystem));
    SmartDashboard.putData("RunWheelsForwardButton", new RunWheelsForwardButton());
    SmartDashboard.putData("RotateWheelsButton", new RotateWheelsButton());
    SmartDashboard.putData("Auto Level No Counter", new AutoLevelNoCounterCommand(driveSubsystem, cannonSubsystem));
    SmartDashboard.putData("Backwards Auto Level No Counter", new BackwardsAutoLevelNoCounterCommand(driveSubsystem, cannonSubsystem));
    SmartDashboard.putData("AutoAlignToChute", new AutoAlignToChuteCommand(driveSubsystem));

    // Autos
    SmartDashboard.putData("Mid1NoPickupBalanceAuto", new Mid1NoPickupBalanceAuto(driveSubsystem, visionSubsystem, cannonSubsystem, odometrySubsystem));
    SmartDashboard.putData("MidCrossoverBalanceAuto", new Mid1NoGrabBalanceAuto(driveSubsystem, visionSubsystem, cannonSubsystem, odometrySubsystem));
    SmartDashboard.putData("Human1PickupBalanceAuto", new Human1PickupBalanceAuto(driveSubsystem, visionSubsystem, cannonSubsystem, odometrySubsystem));
    SmartDashboard.putData("Wall1PickupBalanceAuto", new Wall1PickupBalanceAuto(driveSubsystem, visionSubsystem, cannonSubsystem, odometrySubsystem));
    SmartDashboard.putData("Human2NoPickupNoBalanceAuto", new Human2NoPickupNoBalanceAuto(driveSubsystem, visionSubsystem, cannonSubsystem, odometrySubsystem));
    SmartDashboard.putData("Human1NoPickupBalanceAuto", new Human1NoPickupBalanceAuto(driveSubsystem, visionSubsystem, cannonSubsystem, odometrySubsystem));
    SmartDashboard.putData("Human1BlindPickupBalanceAuto", new Human1BlindPickupBalanceAuto(driveSubsystem, visionSubsystem, cannonSubsystem, odometrySubsystem));
    SmartDashboard.putData("Wall1NoPickupBalanceAuto", new Wall1NoPickupBalanceAuto(driveSubsystem, visionSubsystem, cannonSubsystem, odometrySubsystem));
    SmartDashboard.putData("Wall1BlindPickupBalanceAuto", new Wall1BlindPickupBalanceAuto(driveSubsystem, visionSubsystem, cannonSubsystem, odometrySubsystem));
    SmartDashboard.putData("Wall1BlindPickupBalanceAutoButWeStop", new Wall1BlindPickupBalanceAutoButWeStop(driveSubsystem, visionSubsystem, cannonSubsystem, odometrySubsystem));
    SmartDashboard.putData("Human2BlindNoPickupNoBalanceAuto", new Human2BlindNoPickupNoBalanceAuto(driveSubsystem, visionSubsystem, cannonSubsystem, odometrySubsystem));
    SmartDashboard.putData("Wall2BlindNoPickupNoBalanceAuto", new Wall2BlindNoPickupNoBalanceAuto(driveSubsystem, visionSubsystem, cannonSubsystem, odometrySubsystem));
    SmartDashboard.putData("Human2.5", new Human2BackwardsGrabNoBalanceAuto(driveSubsystem, cannonSubsystem, visionSubsystem, odometrySubsystem));
    SmartDashboard.putData("Wall2.5", new Wall2BackwardsGrabNoBalanceAuto(driveSubsystem, cannonSubsystem, visionSubsystem, odometrySubsystem));
    SmartDashboard.putData("Mid1GrabBalanceAuto", new Mid1GrabBalanceAuto(driveSubsystem, visionSubsystem, cannonSubsystem, odometrySubsystem));
    SmartDashboard.putData("Human2Balance", new Human2BackwardsGrabBalanceAuto(driveSubsystem, cannonSubsystem, visionSubsystem, odometrySubsystem));
    SmartDashboard.putData("Wall2Balance", new Wall2BackwardsGrabBalanceAuto(driveSubsystem, cannonSubsystem, visionSubsystem, odometrySubsystem));

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
    SmartDashboard.putData("RecalibratePitch", new InstantCommand(() -> cannonSubsystem.recalibrataePitch()));
    SmartDashboard.putData("BackwardsHalfwayLocation", new SetCannonLocationCommand(CannonLocation.backwardsHalfwayLocation));
    SmartDashboard.putData("BackwardsFloorPickupLocation", new SetCannonLocationCommand(CannonLocation.backwardsFloorPickupLocation));
    SmartDashboard.putData("halfwayToConeHighLocation", new SetCannonLocationCommand(CannonLocation.halfwayToConeHighLocation));

    // Odometry and Vision Tests
    SmartDashboard.putData(new SeeConeCommand());
    SmartDashboard.putData(new SeeCubeCommand());
    SmartDashboard.putData(new TestPhotonVisionPipelineSwitchCommand());

    // Lights
    SmartDashboard.putData("testWarningLights", new testWarningLightCommand());

    // Autonomous

  }

  SendableChooser<CommandFactory> chooser = new SendableChooser<>();
  public void setupAutonomousCommands() {
    SmartDashboard.putData("Auto mode", chooser);
    chooser.setDefaultOption("Do nothing", () -> new LogCommand("no autonomous specified, did nothing"));
    //chooser.addOption("April Tag Auto Test", () -> new AprilTagAutoTestCommand(driveSubsystem, visionSubsystem));
    chooser.addOption("MidCrossoverBalanceAuto", () -> new Mid1NoGrabBalanceAuto(driveSubsystem, visionSubsystem, cannonSubsystem, odometrySubsystem));
    chooser.addOption("MidOnlyBalanceAuto", () -> new MidOnlyBalanceAuto(driveSubsystem, visionSubsystem, cannonSubsystem, odometrySubsystem));
    //chooser.addOption("Mid1BalanceAuto", () -> new Mid1NoPickupBalanceAuto(driveSubsystem, visionSubsystem, cannonSubsystem, odometrySubsystem));
    //chooser.addOption("Human1PickupBalanceAuto", () -> new Human1PickupBalanceAuto(driveSubsystem, visionSubsystem, cannonSubsystem, odometrySubsystem));
    //chooser.addOption("Human1BlindPickupBalanceAuto", () -> new Human1BlindPickupBalanceAuto(driveSubsystem, visionSubsystem, cannonSubsystem, odometrySubsystem));
    //chooser.addOption("Human1NoPickupBalanceAuto", () -> new Human1NoPickupBalanceAuto(driveSubsystem, visionSubsystem, cannonSubsystem, odometrySubsystem));
    //chooser.addOption("Human2NoPickupNoBalanceAuto", () -> new Human2NoPickupNoBalanceAuto(driveSubsystem, visionSubsystem, cannonSubsystem, odometrySubsystem));
    //chooser.addOption("Human2BlindNoPickupNoBalanceAuto", () -> new Human2BlindNoPickupNoBalanceAuto(driveSubsystem, visionSubsystem, cannonSubsystem, odometrySubsystem));
    //chooser.addOption("Wall1BlindPickupBalanceAuto", () -> new Wall1BlindPickupBalanceAuto(driveSubsystem, visionSubsystem, cannonSubsystem, odometrySubsystem));
    //chooser.addOption("Wall1PickupBalanceAuto", () -> new Wall1PickupBalanceAuto(driveSubsystem, visionSubsystem, cannonSubsystem, odometrySubsystem));
    //chooser.addOption("Wall1NoPickupBalanceAuto", () -> new Wall1NoPickupBalanceAuto(driveSubsystem, visionSubsystem, cannonSubsystem, odometrySubsystem));
    //chooser.addOption("Wall2BlindNoPickupNoBalanceAuto", () -> new Wall2BlindNoPickupNoBalanceAuto(driveSubsystem, visionSubsystem, cannonSubsystem, odometrySubsystem));
    //chooser.addOption("Wall2NoPickupNoBalanceAuto", () -> new Wall2NoPickupNoBalanceAuto(driveSubsystem, visionSubsystem, cannonSubsystem, odometrySubsystem));
    chooser.addOption("Mid1GrabBalanceAuto", () -> new Mid1GrabBalanceAuto(driveSubsystem, visionSubsystem, cannonSubsystem, odometrySubsystem));
    chooser.addOption("Human2NoBalanceAuto", () -> new Human2BackwardsGrabNoBalanceAuto(driveSubsystem, cannonSubsystem, visionSubsystem, odometrySubsystem));
    chooser.addOption("Human2BalanceAuto", () -> new Human2BackwardsGrabBalanceAuto(driveSubsystem, cannonSubsystem, visionSubsystem, odometrySubsystem));
    chooser.addOption("Wall2NoBalanceAuto", () -> new Wall2BackwardsGrabNoBalanceAuto(driveSubsystem, cannonSubsystem, visionSubsystem, odometrySubsystem));
    //chooser.addOption("Mid2BalanceAuto", () -> new Mid2BalanceAuto(driveSubsystem, visionSubsystem, cannonSubsystem, odometrySubsystem));
    chooser.addOption("Wall2BalanceAuto", () -> new Wall2BackwardsGrabBalanceAuto(driveSubsystem, cannonSubsystem, visionSubsystem, odometrySubsystem));
  }

  interface CommandFactory extends Supplier<Command> { }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    CommandFactory factory = chooser.getSelected();
    Command command = factory.get();
    logger.info ("Command Factory gave us a {}", command);
    return command;
  }

  public static double getDriveVerticalJoystick() {
    double axisValue = driverJoystick.getRawAxis(XBoxConstants.AXIS_LEFT_Y, FlySkyConstants.AXIS_LEFT_Y);
    double deadzone = 0.1;
    if(driverJoystick.getCurrentControllerType() == ControllerType.B){
      deadzone = 0.05;
    }
    SmartDashboard.putNumber("driver.y.raw", axisValue);
    if (Math.abs(axisValue) < deadzone) {
      return 0;
    }
    if (axisValue < 0){
      return (axisValue*axisValue);
    }
    return -axisValue*axisValue;
  }

  public static double getDriveHorizontalJoystick() {
    double axisValue = driverJoystick.getRawAxis(XBoxConstants.AXIS_LEFT_X, FlySkyConstants.AXIS_LEFT_X);
    double deadzone = 0.1;
    if(driverJoystick.getCurrentControllerType() == ControllerType.B){
      deadzone = 0.05;
    }
    SmartDashboard.putNumber("driver.x.raw", axisValue);
    if (Math.abs(axisValue) < deadzone) {
      return 0;
    }
    if (axisValue < 0){
      return -(axisValue*axisValue);
    }
    return axisValue*axisValue;
  }

  public static double getDriveSpinJoystick() {
    double axisValue = driverJoystick.getRawAxis(XBoxConstants.AXIS_RIGHT_X, FlySkyConstants.AXIS_RIGHT_X);
    double deadzone = 0.1;
    if(driverJoystick.getCurrentControllerType() == ControllerType.B){
      deadzone = 0.05;
    }
    SmartDashboard.putNumber("driver.spin.raw", axisValue);
    double rv = 0;
    if (Math.abs(axisValue) >= deadzone) {
      rv = axisValue*axisValue;
      if (axisValue < 0){
        rv = -rv;
      }
    }

    /*
     * this should not be necessary, but autonomous code is looking at
     * the spin joystick, so here we are...
     */
    if (Robot.getCurrentRobotMode() == RobotMode.AUTONOMOUS) {
      rv = 0;
    }

    SmartDashboard.putNumber("driver.spin.processed", rv);
    return rv;
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