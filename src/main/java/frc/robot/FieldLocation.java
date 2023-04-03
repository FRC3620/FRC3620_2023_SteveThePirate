// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.usfirst.frc3620.misc.PoseOnField;

/** Add your docs here. */
public class FieldLocation {
    static double yOffset = 0;
    static double blueMidYOffset = -0.15; // use this for location in close
    static double bluePreYOffset = -0.30;
    static double bluePostYOffset = -0.47; // use this for location way out there
    public static PoseOnField humanCommunity = PoseOnField.fromRedAlliancePositionInMeters(14.122, 4.75, blueMidYOffset); //4.800
    public static PoseOnField midCommunityHuman = PoseOnField.fromRedAlliancePositionInMeters(14.422, 3.683, blueMidYOffset);
    public static PoseOnField midCommunityWall = PoseOnField.fromRedAlliancePositionInMeters(14.422, 1.746);
    public static PoseOnField wallCommunity = PoseOnField.fromRedAlliancePositionInMeters(14.122, 0.888, blueMidYOffset);
    public static PoseOnField humanMiddle = PoseOnField.fromRedAlliancePositionInMeters(11.33, 4.990 + yOffset); //4.800
    public static PoseOnField humanMiddlePlus = PoseOnField.fromRedAlliancePositionInMeters(10, 4.990 + yOffset); //4.800
    public static PoseOnField midMiddle = PoseOnField.fromRedAlliancePositionInMeters(11.4, 2.203 + yOffset);
    public static PoseOnField wallMiddle = PoseOnField.fromRedAlliancePositionInMeters(11.33, 0.963 + yOffset);

    public static PoseOnField humanStart = PoseOnField.fromRedAlliancePositionInMeters(14.829, 4.986 + yOffset); //TargetPoseOnField.humanTarget(-0.5, Units.inchesToMeters(22));
    public static PoseOnField midStart = PoseOnField.fromRedAlliancePositionInMeters(14.829, 2.203 + yOffset); //TargetPoseOnField.midTarget(-0.5, Units.inchesToMeters(22)); ........y was 3.402
    public static PoseOnField wallStart = PoseOnField.fromRedAlliancePositionInMeters(14.829, 0.651 + yOffset); //TargetPoseOnField.wallTarget(-0.5, Units.inchesToMeters(22));

    public static PoseOnField humanHalfway = PoseOnField.fromRedAlliancePositionInMeters(12.876, 4.7 + yOffset); //4.800
    public static PoseOnField wallHalfway = PoseOnField.fromRedAlliancePositionInMeters(12.576, 0.763 + yOffset);

    public static PoseOnField humanBlindPosition = PoseOnField.fromRedAlliancePositionInMeters(10.789, 4.65 + yOffset); //4.138
    public static PoseOnField humanMiddleBlind = PoseOnField.fromRedAlliancePositionInMeters(12.1, 4.990 + yOffset);
    public static PoseOnField wallBlindPosition = PoseOnField.fromRedAlliancePositionInMeters(10.024, 1.053 + yOffset);
    public static PoseOnField wallMiddleBlind = PoseOnField.fromRedAlliancePositionInMeters(12.1, 0.963 + yOffset);

    public static PoseOnField humanPickupBehindPre = PoseOnField.fromRedAlliancePositionInMeters(11.768, 4.927, bluePreYOffset);
    public static PoseOnField humanPickupBehindPost = PoseOnField.fromRedAlliancePositionInMeters(10.968, 4.927, bluePostYOffset);
    // don't do a blue offset here, you could snag the charging station
    public static PoseOnField wallPickupBehindPre = PoseOnField.fromRedAlliancePositionInMeters(11.968, 0.75);
    public static PoseOnField wallPickupBehindPost = PoseOnField.fromRedAlliancePositionInMeters(10.968, 1.18, bluePostYOffset);
    public static PoseOnField midPickupBehindPre = PoseOnField.fromRedAlliancePositionInMeters(11.450, 2.345);
    public static PoseOnField midPickupBehindPost = PoseOnField.fromRedAlliancePositionInMeters(10.968, 2.345);
    
    public static PoseOnField humanPlaceCube = PoseOnField.fromRedAlliancePositionInMeters(14.85, 4.44);
    public static PoseOnField wallPlaceCube = PoseOnField.fromRedAlliancePositionInMeters(14.85, 1.047);
    public static PoseOnField midPlaceCube = PoseOnField.fromRedAlliancePositionInMeters(14.85,2.869);

    public static PoseOnField humanGrabSecondPiece = PoseOnField.fromRedAlliancePositionInMeters(10.108, 3.50); //x was 10.708

    /*public static PoseOnField humanPickupBehindPreBlue = PoseOnField.fromRedAlliancePositionInMeters(11.768, 4.777 + blueOffset + yOffset);
    public static PoseOnField humanPickupBehindPostBlue = PoseOnField.fromRedAlliancePositionInMeters(10.968, 4.777 + blueOffset + yOffset);
    public static PoseOnField humanGrabSecondPieceBlue = PoseOnField.fromRedAlliancePositionInMeters(10.108, 3.50 + blueOffset); //x was 10.708

    public static PoseOnField wallPickupBehindPreBlue = PoseOnField.fromRedAlliancePositionInMeters(11.968, 1.05 + blueOffset + yOffset);
    public static PoseOnField wallPickupBehindPostBlue = PoseOnField.fromRedAlliancePositionInMeters(10.968, 1.05 + blueOffset + yOffset);*/
}
