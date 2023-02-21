import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.CannonLocation;
import frc.robot.LegoAssembler;
import frc.robot.RobotContainer;
import frc.robot.TargetPoseOnField;
import frc.robot.TargetPoseOnField.TargetPosition;
import frc.robot.commands.CannonElevateCommand;
import frc.robot.commands.CannonExtendCommand;
import frc.robot.commands.CannonPitchCommand;
import frc.robot.commands.DriveToCoordinateCommand;
import frc.robot.commands.SetCannonClawSpeedCommand;
import frc.robot.commands.SetCannonLocationCommand;
import frc.robot.subsystems.CannonSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class LegoAssemblerTest {

    @Test
    public void test00() {
        VisionSubsystem visionSubsystem = new VisionSubsystem();
        SuperJoe sj = new SuperJoe();
        sj.assemble(TargetPosition.MID, 0, false, false);

        for (var step: sj.getReasonsWithCommands()) {
            System.out.println(step);
        }

    }

    class SuperJoe extends LegoAssembler {
        SuperJoe() {
            super();
        }

        @Override
        protected void assemble (TargetPosition startPosition, int gamepieces, boolean placeLast, boolean balance) {
            new RobotContainer();
            int targetId = startPosition.getTargetPoseOnField().getTargetId();

            DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
        
            DriveSubsystem driveSubsystem = RobotContainer.driveSubsystem;
            CannonSubsystem cannonSubsystem = RobotContainer.cannonSubsystem;
            add("Alliance: " + DriverStation.getAlliance());
            add("Start Position: " + startPosition);
            add("Target Id: " + targetId);

            add("reset navx");
            add("check odometry");

            //add(new DriveToCoordinateCommand(startPosition.getTargetPoseOnField(), 0.1, 0.1, 0, driveSubsystem));

            //mostly placeholder numbers, CANNONLOCATION NUMBERS ARE NOT TESTED

            add(new DriveToCoordinateCommand(TargetPoseOnField.target(startPosition, -1, -1), 0.1, 0.1, 0, driveSubsystem), "Move in front of peg");
            add(new SetCannonLocationCommand(CannonLocation.coneHighLocation),"Set Cannon to deposit position");
            add(new SetCannonClawSpeedCommand(cannonSubsystem, -0.1), "Deposit Game Piece");
            add(new SetCannonLocationCommand(CannonLocation.parkLocation),"Set Cannon to vertical position");
            add(new DriveToCoordinateCommand(TargetPoseOnField.target(startPosition, -5, -1), 0.1, 0.1, 180, driveSubsystem), "Exit community");
            
            //extra in case we get this far
            add(new SetCannonLocationCommand(CannonLocation.floorPickLocation),"Set Cannon to pickup position");
            add(new SetCannonClawSpeedCommand(cannonSubsystem, 0.2), "Pickup Game Piece");
            add(new SetCannonLocationCommand(CannonLocation.parkLocation),"Set Cannon to vertical position");
            add(new DriveToCoordinateCommand(TargetPoseOnField.target(startPosition, -1, -1), 0.1, 0.1, 0, driveSubsystem), "Move in front of peg");
            add(new SetCannonLocationCommand(CannonLocation.coneHighLocation),"Set Cannon to deposit position");
            add(new SetCannonClawSpeedCommand(cannonSubsystem, -0.1), "Deposit Game Piece");
            add(new SetCannonLocationCommand(CannonLocation.parkLocation),"Set Cannon to vertical position");

            //Whatever comes next

        }
    }
}
