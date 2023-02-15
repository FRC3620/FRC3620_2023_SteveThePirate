package frc.robot;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.commands.*;

import frc.robot.subsystems.VisionSubsystem;

import java.util.Set;

import org.slf4j.Logger;
import org.usfirst.frc3620.logger.EventLogging;
import org.usfirst.frc3620.logger.LogCommand;
import org.usfirst.frc3620.logger.EventLogging.Level;
import org.usfirst.frc3620.misc.CANDeviceFinder;

import frc.robot.subsystems.CannonSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.INavigationSubsystem;
import frc.robot.subsystems.NavXNavigationSubsystem;
import frc.robot.subsystems.OdometrySubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import org.usfirst.frc3620.misc.CANDeviceType;
import org.usfirst.frc3620.misc.JoystickAnalogButton;
import org.usfirst.frc3620.misc.PoseOnField;
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

    new JoystickButton(driverJoystick, XBoxConstants.BUTTON_A)
            .whileTrue(new XModeCommand(driveSubsystem));

    new JoystickButton(driverJoystick, XBoxConstants.BUTTON_X)
            .onTrue(new ResetNavXCommand());
    
    
    new JoystickButton(operatorJoystick, XBoxConstants.BUTTON_RIGHT_BUMPER)
            .onTrue(new SetCannonLocationCommand(CannonLocation.coneHighLocation));
    new JoystickAnalogButton(operatorJoystick, XBoxConstants.AXIS_RIGHT_TRIGGER)
            .onTrue(new SetCannonLocationCommand(CannonLocation.coneMidLocation));
    new JoystickButton(operatorJoystick, XBoxConstants.BUTTON_LEFT_BUMPER)
            .onTrue(new SetCannonLocationCommand(CannonLocation.cubeHighLocation));
    new JoystickAnalogButton(operatorJoystick, XBoxConstants.AXIS_LEFT_TRIGGER)
            .onTrue(new SetCannonLocationCommand(CannonLocation.cubeMidLocation));
    new JoystickButton(operatorJoystick, XBoxConstants.BUTTON_A)
            .onTrue(new SetCannonLocationCommand(CannonLocation.lowLocation));
  }

  private void setupSmartDashboardCommands() 
  {
    SmartDashboard.putData("Strafe to target", new StrafeToAprilTagCommand(driveSubsystem, visionSubsystem));
    SmartDashboard.putData("Move to target", new LocateAprilTagCommand(driveSubsystem, visionSubsystem));
    SmartDashboard.putData("Updated Move to April Tag", new UpdatedLocateAprilTagCommand(driveSubsystem, visionSubsystem));
    SmartDashboard.putData("AprilTagAutoTestCommand", new AprilTagAutoTestCommand(driveSubsystem, visionSubsystem));
    SmartDashboard.putData("Drive to coordinate", new DriveToCoordinateCommand(PoseOnField.fromRedAlliancePositionInMeters(10.8, 4.7), 0.2, 0.1, driveSubsystem));
    SmartDashboard.putData("Test Coordinate Auto", new TestCoordinateAuto(driveSubsystem));
    SmartDashboard.putData("Longer Test Coordinate Auto", new LongerTestCoordinateAuto(driveSubsystem));
    SmartDashboard.putData("RunWheelsForwardButton", new RunWheelsForwardButton());
    SmartDashboard.putData(" RotateWheelsButton", new RotateWheelsButton());
    SmartDashboard.putBoolean("DiagnosticsDriveMotortest", true);
    SmartDashboard.putData("ExtendCommand1" , new CannonExtendCommand(cannonSubsystem, 3));
    SmartDashboard.putData("ExtendCommand2" , new CannonExtendCommand(cannonSubsystem, 15));
    SmartDashboard.putData("ElevateCommand1", new CannonElevateCommand(cannonSubsystem, 21));
    SmartDashboard.putData("ElevateCommand2", new CannonElevateCommand(cannonSubsystem, 0));
    SmartDashboard.putData("ElevateHome", new CannonElevateCommand(cannonSubsystem, 65));
    SmartDashboard.putData("RollCommand1", new CannonRollCommand(cannonSubsystem, 12));
    SmartDashboard.putData("RollCommand2", new CannonRollCommand(cannonSubsystem, 5));
    SmartDashboard.putData("PitchCommand1", new CannonPitchCommand(cannonSubsystem, 15));
    SmartDashboard.putData("PitchCommand2", new CannonPitchCommand(cannonSubsystem, -15));
    SmartDashboard.putData("HighLocation", new SetCannonLocationCommand(CannonLocation.coneHighLocation));
    SmartDashboard.putData("MidLocation", new SetCannonLocationCommand(CannonLocation.coneMidLocation));
    SmartDashboard.putData("ParkLocation", new SetCannonLocationCommand(CannonLocation.parkLocation));
  }

  SendableChooser<Command> chooser = new SendableChooser<>();
  public void setupAutonomousCommands() {
    SmartDashboard.putData("Auto mode", chooser);
    chooser.setDefaultOption("Do nothing", new LogCommand("no autonomous specified, did nothing"));
    chooser.addOption("April Tag Auto Test", new AprilTagAutoTestCommand(driveSubsystem, visionSubsystem));
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

    if(practiceBotJumper.get() == true){
      return true;
    }

    if (robotParameters.shouldMakeAllCANDevices()) {
      return true;
    }

    return false;
  }

}