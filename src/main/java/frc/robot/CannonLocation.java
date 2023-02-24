// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


/** Add your docs here. */
public class CannonLocation {
    public static CannonLocation coneHighLocation = new CannonLocation(53, 33.5, -61);
    public static CannonLocation cubeHighLocation = new CannonLocation(48.8, 25, -51.3);
    public static CannonLocation coneMidLocation = new CannonLocation(47, 13.8, -70);
    public static CannonLocation cubeMidLocation = new CannonLocation(47.8, 7.8, -59.6);
    public static CannonLocation lowLocation = new CannonLocation(3.5, 0, -67);
    public static CannonLocation floorPickLocation = new CannonLocation(-3.8, 0, -33.26);
    public static CannonLocation humanLocation = new CannonLocation(60, 8, -40);
    public static CannonLocation parkLocation = new CannonLocation(90, 1, -90);
    public static CannonLocation sidewaysConeLocation = new CannonLocation(0, 0, 8);
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
