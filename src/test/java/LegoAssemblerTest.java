import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.LegoAssembler;
import frc.robot.RobotContainer;
import frc.robot.TargetPoseOnField;
import frc.robot.TargetPoseOnField.TargetPosition;
import frc.robot.commands.DriveToCoordinateCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class LegoAssemblerTest {

    @Test
    public void test00() {
        VisionSubsystem visionSubsystem = new VisionSubsystem();
        SuperJoe sj = new SuperJoe();
        sj.assemble(TargetPosition.HUMAN, 0, false, false);

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
            add("Alliance: " + DriverStation.getAlliance());
            add("Start Position: " + startPosition);
            add(new DriveToCoordinateCommand(startPosition.getTargetPoseOnField(), 0.1, 0.1, 0, driveSubsystem));
            add(new DriveToCoordinateCommand(TargetPoseOnField.target(startPosition, -1, -1), 0.1, 0.1, 0, driveSubsystem));
            add("Target Id: " + targetId);
        }
    }
}
