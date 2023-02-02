import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import org.usfirst.frc3620.misc.PoseOnField;

import java.io.IOException;

public class PoseOnFieldTest {
  @Test
  public void checkCenterLine() throws IOException {
    AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);

    testAverageXOfTargets(fieldLayout, 1, 8);
    testAverageXOfTargets(fieldLayout, 2, 7);
    testAverageXOfTargets(fieldLayout, 3, 6);
    testAverageXOfTargets(fieldLayout, 4, 5);
  }

  void testAverageXOfTargets (AprilTagFieldLayout fieldLayout, int a, int b) {
    Pose3d pa = fieldLayout.getTagPose(a).get();
    Pose3d pb = fieldLayout.getTagPose(b).get();
    Assertions.assertEquals(Constants.FIELD_LENGTH_IN_METERS, pa.getTranslation().getX() + pb.getTranslation().getX(), 0.01);

  }

  @Test
  public void testCenterLine() {
    testOneCenterLine(1, 0);
    testOneCenterLine(-1, 0);
    testOneCenterLine(-1, 1);
    testOneCenterLine(1, 1);
  }

  void testOneCenterLine(double x, double y) {
    PoseOnField p = PoseOnField.makePoseFromCenterLineInMeters(new Translation2d(x, y));

    Translation2d rt = p.getTranslationInMeters(DriverStation.Alliance.Red);
    Assertions.assertEquals (Constants.FIELD_LENGTH_IN_METERS/2 + x, rt.getX());
    Assertions.assertEquals(y, rt.getY());

    Translation2d bt = p.getTranslationInMeters(DriverStation.Alliance.Blue);
    Assertions.assertEquals (Constants.FIELD_LENGTH_IN_METERS/2 - x, bt.getX());
    Assertions.assertEquals(y, bt.getY());
  }

  @Test
  public void testFromWallLine() {
    testOneWall(1, 0);
    testOneWall(-1, 0);
    testOneWall(-1, 1);
    testOneWall(1, 1);
  }

  void testOneWall(double x, double y) {
    PoseOnField p = PoseOnField.makePoseFromMyWallInMeters(new Translation2d(x, y));

    Translation2d rt = p.getTranslationInMeters(DriverStation.Alliance.Red);
    Assertions.assertEquals (Constants.FIELD_LENGTH_IN_METERS - x, rt.getX());
    Assertions.assertEquals(y, rt.getY());

    Translation2d bt = p.getTranslationInMeters(DriverStation.Alliance.Blue);
    Assertions.assertEquals (x, bt.getX());
    Assertions.assertEquals(y, bt.getY());

  }

}
