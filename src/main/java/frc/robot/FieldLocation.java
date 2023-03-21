// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.usfirst.frc3620.misc.PoseOnField;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class FieldLocation {
    static double yOffset = 0;
    public static PoseOnField humanCommunity = PoseOnField.fromRedAlliancePositionInMeters(14.422,4.60 + yOffset); //4.800
    public static PoseOnField midCommunity = PoseOnField.fromRedAlliancePositionInMeters(14.422, 3.458 + yOffset);
    public static PoseOnField wallCommunity = PoseOnField.fromRedAlliancePositionInMeters(14.422, 0.963 + yOffset);
    public static PoseOnField humanMiddle = PoseOnField.fromRedAlliancePositionInMeters(11.33, 4.990 + yOffset); //4.800
    public static PoseOnField humanMiddlePlus = PoseOnField.fromRedAlliancePositionInMeters(10, 4.990 + yOffset); //4.800
    public static PoseOnField midMiddle = PoseOnField.fromRedAlliancePositionInMeters(11.33, 3.058 + yOffset);
    public static PoseOnField wallMiddle = PoseOnField.fromRedAlliancePositionInMeters(11.33, 0.963 + yOffset);

    public static PoseOnField humanStart = PoseOnField.fromRedAlliancePositionInMeters(14.829, 4.986 + yOffset); //TargetPoseOnField.humanTarget(-0.5, Units.inchesToMeters(22));
    public static PoseOnField midStart = PoseOnField.fromRedAlliancePositionInMeters(14.829, 3.402 + yOffset); //TargetPoseOnField.midTarget(-0.5, Units.inchesToMeters(22));
    public static PoseOnField wallStart = PoseOnField.fromRedAlliancePositionInMeters(14.829, 0.651 + yOffset); //TargetPoseOnField.wallTarget(-0.5, Units.inchesToMeters(22));

    public static PoseOnField humanHalfway = PoseOnField.fromRedAlliancePositionInMeters(12.876, 4.7 + yOffset); //4.800
    public static PoseOnField wallHalfway = PoseOnField.fromRedAlliancePositionInMeters(12.576, 0.763 + yOffset);

    public static PoseOnField humanBlindPosition = PoseOnField.fromRedAlliancePositionInMeters(10.789, 4.65 + yOffset); //4.138
    public static PoseOnField humanMiddleBlind = PoseOnField.fromRedAlliancePositionInMeters(12.1, 4.990 + yOffset);
    public static PoseOnField wallBlindPosition = PoseOnField.fromRedAlliancePositionInMeters(10.024, 1.053 + yOffset);
    public static PoseOnField wallMiddleBlind = PoseOnField.fromRedAlliancePositionInMeters(12.1, 0.963 + yOffset);

    public static PoseOnField humanPickupBehindPre = PoseOnField.fromRedAlliancePositionInMeters(11.968, 4.777 + yOffset);
    public static PoseOnField humanPickupBehindPost = PoseOnField.fromRedAlliancePositionInMeters(10.968, 4.777 + yOffset);
    public static PoseOnField wallPickupBehindPre = PoseOnField.fromRedAlliancePositionInMeters(11.968, 0.95 + yOffset);
    public static PoseOnField wallPickupBehindPost = PoseOnField.fromRedAlliancePositionInMeters(10.968, 0.95 + yOffset);
    
    public static PoseOnField humanPlaceCube = PoseOnField.fromRedAlliancePositionInMeters(14.85, 4.44);
    public static PoseOnField wallPlaceCube = PoseOnField.fromRedAlliancePositionInMeters(14.800, 1.047); //x = 14.847
}

