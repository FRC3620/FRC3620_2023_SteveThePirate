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
    public static PoseOnField humanMiddlePlus = PoseOnField.fromRedAlliancePositionInMeters(10, 4.800);
    public static PoseOnField midMiddle = PoseOnField.fromRedAlliancePositionInMeters(11.33, 3.458);
    public static PoseOnField wallMiddle = PoseOnField.fromRedAlliancePositionInMeters(11.33, 0.963);

    public static PoseOnField humanStart = PoseOnField.fromRedAlliancePositionInMeters(14.829, 4.986); //TargetPoseOnField.humanTarget(-0.5, Units.inchesToMeters(22));
    public static PoseOnField midStart = PoseOnField.fromRedAlliancePositionInMeters(14.829, 3.402); //TargetPoseOnField.midTarget(-0.5, Units.inchesToMeters(22));
    public static PoseOnField wallStart = PoseOnField.fromRedAlliancePositionInMeters(14.829, 0.651); //TargetPoseOnField.wallTarget(-0.5, Units.inchesToMeters(22));

    public static PoseOnField humanHalfway = PoseOnField.fromRedAlliancePositionInMeters(12.876,4.8);
    public static PoseOnField wallHalfway = PoseOnField.fromRedAlliancePositionInMeters(12.876, 0.963);
}
