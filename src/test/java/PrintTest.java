import org.junit.jupiter.api.Test;

import org.usfirst.frc3620.misc.PoseOnField;
import frc.robot.TargetPoseOnField;
import frc.robot.subsystems.VisionSubsystem;

public class PrintTest {

    @Test
    public void test00() {
        // need this because of fuzzy thinking
        var v = new VisionSubsystem();
        
        PoseOnField p = TargetPoseOnField.HumanTarget;
        System.out.println(p);
    }

}
