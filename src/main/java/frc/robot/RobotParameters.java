package frc.robot;

import org.usfirst.frc3620.misc.RobotParametersBase;
import org.usfirst.frc3620.misc.SwerveParameters;

/**
 * add members here as needed
 */
public class RobotParameters extends RobotParametersBase {
    SwerveParameters swerveParameters;
    int elevationEncoderValueAt90Degrees;

    public SwerveParameters getSwerveParameters() {
        return swerveParameters;
    }

    public int getElevationEncoderValueAt90Degrees() {
        return elevationEncoderValueAt90Degrees;
    }

    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder(super.toString());
        sb.setLength(sb.length()-1);
        sb.append (", swerveParameters=" + swerveParameters);
        sb.append (", elevation encoder at 90 degrees=" + elevationEncoderValueAt90Degrees);
        sb.append ("]");
        return sb.toString();
    }

}