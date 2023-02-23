// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


/** Add your docs here. */
public class CannonLocation {
    public static CannonLocation coneHighLocation = new CannonLocation(42.9, 31.8, -55.8);
    public static CannonLocation cubeHighLocation = new CannonLocation(42.9, 31.8, -55.8);
    public static CannonLocation coneMidLocation = new CannonLocation(44.3, 15.8, -80);
    public static CannonLocation cubeMidLocation = new CannonLocation(33.75, 10, -44);
    public static CannonLocation lowLocation = new CannonLocation(3.5, 0, -67);
    public static CannonLocation floorPickLocation = new CannonLocation(-10, 0, -10);
    public static CannonLocation humanLocation = new CannonLocation(60, 8, -40);
    public static CannonLocation parkLocation = new CannonLocation(90, 3, 0);
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
