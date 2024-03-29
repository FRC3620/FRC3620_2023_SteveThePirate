import static org.junit.Assert.assertEquals;

import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.ExpectedException;

import org.usfirst.frc3620.misc.DriveVectors;
import org.usfirst.frc3620.misc.SwerveCalculator;
import org.usfirst.frc3620.misc.Vector;

public class SwerveCalcTest {
  SwerveCalculator sc = new SwerveCalculator(1, 1, 1);

  @Test
  public void s00() {
    sw(0, 1, 0);
    sw(1, 0, 90);
    sw(1, 1, 45);
  }

  void sw (double x, double y, double expected) {
    double sa = SwerveCalculator.calculateStrafeAngle(x, y);
    assertEquals("swerve calculation broke", expected, sa, 0.1);
  }

  @Test
  public void t00() {
    xy (0, 1, 90, 1);
    xy (1, 0, 0, 1);
    xy (1, 1, 45, 1);
  }

  @Test
  public void t01() {
    spin(1.0);
    spin(0.0);
    spin(-1.0);
  }

  @Rule
  public ExpectedException exceptionRule = ExpectedException.none();

  @Test
  public void t02_shouldFail() {
    exceptionRule.expect(AssertionError.class);
    xy (1, 1, 45, 2); // should fail
  }

  @Test
  public void t03_shouldFail() {
    exceptionRule.expect(AssertionError.class);
    xy (1, 1, 47, 1); // should fail
  }

  void spin (double spin) {
    DriveVectors dv = sc.calculateEverything(0, 0, spin);
    System.out.println ("spin: " + spin + " -> " + dv);
  }

  void xy (double x, double y, double expected_dv_direction, double expected_dv_magnitude) {
    DriveVectors dv = sc.calculateEverything(x, y, 0);
    System.out.println ("xy: " + x + ", " + y + " -> " + dv);
    checkAllDvs(dv, expected_dv_direction, expected_dv_magnitude);
  }

  void checkOneDv (String s, Vector v, double expected_dv_direction, double expected_dv_magnitude) {
    assertEquals(s + " direction is wrong", expected_dv_direction, v.getDirection(), 0.1);
    assertEquals(s + " magnitude is wrong", expected_dv_magnitude, v.getMagnitude(), 0.02);
  }

  void checkAllDvs (DriveVectors dv, double expected_dv_direction, double expected_dv_magnitude) {
    checkOneDv ("left front", dv.leftFront, expected_dv_direction, expected_dv_magnitude);
    checkOneDv ("right front", dv.rightFront, expected_dv_direction, expected_dv_magnitude);
    checkOneDv ("left back", dv.leftBack, expected_dv_direction, expected_dv_magnitude);
    checkOneDv ("right back", dv.rightBack, expected_dv_direction, expected_dv_magnitude);
  }
}
