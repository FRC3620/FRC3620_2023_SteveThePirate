import org.junit.jupiter.api.Test;

import org.usfirst.frc3620.misc.PoseOnField;
import frc.robot.TargetPoseOnField;
import frc.robot.subsystems.VisionSubsystem;

public class PrintTest {

    @Test
    public void test00() {
        // TODO need this because of fuzzy thinking
        new VisionSubsystem();
        
        PoseOnField p = TargetPoseOnField.humanTarget();
        System.out.println(p);
    }

}
