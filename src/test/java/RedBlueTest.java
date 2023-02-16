import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.Test;

import org.usfirst.frc3620.misc.PoseOnField;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.TargetPoseOnField;
import frc.robot.subsystems.VisionSubsystem;

public class RedBlueTest {

    @Test
    public void test00() {
        // TODO need this because of fuzzy thinking
        new VisionSubsystem();

        List<TargetPoseOnField> poses = new ArrayList<>();
        poses.add(TargetPoseOnField.humanTarget());
        poses.add(TargetPoseOnField.humanTarget(-1, -1));
        poses.add(TargetPoseOnField.humanTarget(-1, 0));
        poses.add(TargetPoseOnField.humanTarget(-1, 1));
        poses.add(TargetPoseOnField.midTarget());
        poses.add(TargetPoseOnField.midTarget(-1, -1));
        poses.add(TargetPoseOnField.midTarget(-1, 0));
        poses.add(TargetPoseOnField.midTarget(-1, 1));
        poses.add(TargetPoseOnField.wallTarget());
        poses.add(TargetPoseOnField.wallTarget(-1, -1));
        poses.add(TargetPoseOnField.wallTarget(-1, 0));
        poses.add(TargetPoseOnField.wallTarget(-1, 1));

        for (var pose: poses) {
            System.out.println (DriverStation.Alliance.Red + ": " + pose.getTargetId(DriverStation.Alliance.Red) + " " + pose.getTranslationInMeters(DriverStation.Alliance.Red));
            System.out.println (DriverStation.Alliance.Blue + ": " + pose.getTargetId(DriverStation.Alliance.Blue) + " " + pose.getTranslationInMeters(DriverStation.Alliance.Blue));
        }
    }
}
