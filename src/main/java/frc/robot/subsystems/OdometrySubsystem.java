package frc.robot.subsystems;

import java.util.function.Supplier;

import org.usfirst.frc3620.misc.SwerveParameters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OdometrySubsystem extends SubsystemBase {

    final SwerveDriveOdometry sdo;
    final INavigationSubsystem navigationSubsystem;
    final Supplier<SwerveModulePosition[]> modulePositionProvider;

    final SwerveDriveKinematics kinematics;

    SwerveDriveOdometry blind_sdo;

    static public boolean putSwerveModulePositionsOnDashboard = false;

    public OdometrySubsystem (INavigationSubsystem ns, Alliance alliance, SwerveParameters swerveParameters, Supplier<SwerveModulePosition[]> modulePositionProvider) {
        this.navigationSubsystem = ns;
        this.modulePositionProvider = modulePositionProvider;

        double halfChassisWidthInMeters = 1;
        double halfChassisLengthInMeters = 1;
        if (swerveParameters != null) {
            if (swerveParameters.getChassisWidth() != null) halfChassisWidthInMeters = Units.inchesToMeters(swerveParameters.getChassisWidth()) / 2.0;
            if (swerveParameters.getChassisLength() != null) halfChassisLengthInMeters = Units.inchesToMeters(swerveParameters.getChassisLength()) / 2.0;
        }
        // +x is towards the front of the robot, +y is towards the left of the robot
        kinematics = new SwerveDriveKinematics(
            new Translation2d(+halfChassisLengthInMeters, +halfChassisWidthInMeters) // LF
            ,
            new Translation2d(+halfChassisLengthInMeters, -halfChassisWidthInMeters) // RF
            ,
            new Translation2d(-halfChassisLengthInMeters, +halfChassisWidthInMeters) // LB
            ,
            new Translation2d(-halfChassisLengthInMeters, -halfChassisWidthInMeters) // RB
        );

        sdo = new SwerveDriveOdometry(kinematics, getOdometryHeading(alliance), modulePositionProvider.get());
    }

    public void resetPosition (Alliance alliance, Translation2d currentPosition) {
        Rotation2d r2d = getOdometryHeading(alliance);
        Pose2d pose2d = new Pose2d(currentPosition, r2d);
        SwerveModulePosition[] sp = getPositions();
        sdo.resetPosition(r2d, sp, pose2d);
        if (blind_sdo == null) {
            blind_sdo = new SwerveDriveOdometry(kinematics, r2d, sp);
            blind_sdo.resetPosition(r2d, sp, pose2d);  // perhaps superfluous?
        } else {
            blind_sdo.resetPosition(r2d, sp, pose2d);
        }
    }

    SwerveModulePosition[] getPositions() {
        SwerveModulePosition[] sp = modulePositionProvider.get();
        if (putSwerveModulePositionsOnDashboard) {
            for (int i = 0; i < sp.length; i++) {
                SmartDashboard.putNumber("odometry.sp." + i + ".angle", sp[i].angle.getDegrees());
                SmartDashboard.putNumber("odometry.sp." + i + ".pos", sp[i].distanceMeters);
            }
        }
        return sp;
    }

    public Pose2d getPoseMeters() {
        return sdo.getPoseMeters();
    }

    public Pose2d update(Alliance alliance) {
        SwerveModulePosition[] sp = getPositions();
        Rotation2d heading = getOdometryHeading(alliance);
        if (blind_sdo != null) blind_sdo.update(heading, sp);
        return sdo.update(heading, sp);
    }

    @Override
    public void periodic() {
        Pose2d whereIIs = update(DriverStation.getAlliance());
        Translation2d whereIIsInInches = whereIIs.getTranslation().div(Units.metersToInches(1));
        SmartDashboard.putNumber("odometry.x", whereIIs.getX());
        SmartDashboard.putNumber("odometry.y", whereIIs.getY());
        SmartDashboard.putNumber("odometry.x inches", whereIIsInInches.getX());
        SmartDashboard.putNumber("odometry.y inches", whereIIsInInches.getY());
    }

    public Rotation2d getOdometryHeading(Alliance alliance) {
        Rotation2d rv = navigationSubsystem.getOdometryHeading(alliance);
        SmartDashboard.putNumber ("odometry.heading", rv.getDegrees());
        return rv;
    }

    Pose2d zeroPose = new Pose2d(0, 0, new Rotation2d());
    public Pose2d getBlindPoseMeters() {
        if (blind_sdo != null) return blind_sdo.getPoseMeters();
        return zeroPose;
    }
    
}
