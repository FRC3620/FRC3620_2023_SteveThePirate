// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


/** Add your docs here. */
public class CannonLocation {
    public static CannonLocation coneHighLocation = new CannonLocation(40.1, 24, -40.1);
    public static CannonLocation coneMidLocation = new CannonLocation(20.4, 12, -20.4);
    public static CannonLocation cubeHighLocation = new CannonLocation(40.1, 24, -40.1);
    public static CannonLocation cubeMidLocation = new CannonLocation(20.4, 12, -20.4);
    public static CannonLocation lowLocation = new CannonLocation(-10, 0, 10);
    public static CannonLocation floorPickLocation = new CannonLocation(-10, 0, -10);
    public static CannonLocation humanLocation = new CannonLocation(10, 8, -20.4);
    public static CannonLocation parkLocation = new CannonLocation(90, 3, 0);
    double elevation;
    double extension;
    double wristPitch;

    public CannonLocation(double elevation, double extension, double wristPitch) {
        this.elevation = elevation;
        this.extension = extension;
        this.wristPitch = wristPitch;
    }

    public double getElevation() {
        return elevation;
    }

    public double getWristPitch() {
        return wristPitch;
    }

    public double getExtension() {
        return extension;
    }
}
