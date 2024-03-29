import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;

public class DriverStationAllianceTest {

    @Test
    public void test00() throws InterruptedException {
        HAL.initialize(500, 0);

        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
        DriverStationSim.notifyNewData();
        System.out.println (DriverStation.getAlliance() + " " + DriverStation.getLocation());

        DriverStationSim.setAllianceStationId(AllianceStationID.Red3);
        DriverStationSim.notifyNewData();
        System.out.println (DriverStation.getAlliance() + " " + DriverStation.getLocation());
    }
}
