// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.usfirst.frc3620.misc.PoseOnField;

/** Add your docs here. */
public class FieldLocation {
    public static PoseOnField humanStart = PoseOnField.fromRedAlliancePositionInMeters(0,0);
    public static PoseOnField midStart = PoseOnField.fromRedAlliancePositionInMeters(0, 0);
    public static PoseOnField wallStart = PoseOnField.fromRedAlliancePositionInMeters(0, 0);
    public static PoseOnField humanMiddle = PoseOnField.fromRedAlliancePositionInMeters(0, 0);
    public static PoseOnField wallMiddle = PoseOnField.fromRedAlliancePositionInMeters(0, 0);
    public static PoseOnField otherSide = PoseOnField.fromRedAlliancePositionInMeters(10.9, 3.3);
}
