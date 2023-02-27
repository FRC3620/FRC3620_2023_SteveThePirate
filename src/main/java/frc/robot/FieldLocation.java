// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.usfirst.frc3620.misc.PoseOnField;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class FieldLocation {
    public static PoseOnField humanCommunity = PoseOnField.fromRedAlliancePositionInMeters(14.422,4.800);
    public static PoseOnField midCommunity = PoseOnField.fromRedAlliancePositionInMeters(14.422, 3.458);
    public static PoseOnField wallCommunity = PoseOnField.fromRedAlliancePositionInMeters(14.422, 0.963);
    public static PoseOnField humanMiddle = PoseOnField.fromRedAlliancePositionInMeters(11.33, 4.800);
    public static PoseOnField midMiddle = PoseOnField.fromRedAlliancePositionInMeters(11.33, 3.458);
    public static PoseOnField wallMiddle = PoseOnField.fromRedAlliancePositionInMeters(11.33, 0.963);

    public static PoseOnField humanStart = TargetPoseOnField.humanTarget(-0.5, Units.inchesToMeters(22));
    public static PoseOnField midStart = TargetPoseOnField.midTarget(-0.5, Units.inchesToMeters(22));
    public static PoseOnField wallStart = TargetPoseOnField.wallTarget(-0.5, Units.inchesToMeters(22));
}
