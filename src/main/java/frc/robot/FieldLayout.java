package frc.robot;

import java.io.IOException;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

public class FieldLayout {

    static private AprilTagFieldLayout aprilTag2023FieldLayout = null;

    public static AprilTagFieldLayout getAprilTag2023FieldLayout() throws IOException {
        if (aprilTag2023FieldLayout == null) {
            aprilTag2023FieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        }
        return aprilTag2023FieldLayout;
    }
}