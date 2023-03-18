// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


/** Add your docs here. */
public class CannonLocation {
    public static CannonLocation coneHighLocation = new CannonLocation(41.69, 32.8, -67.1);
    public static CannonLocation cubeHighLocation = new CannonLocation(40.21, 24.33, -49.5);
    public static CannonLocation coneMidLocation = new CannonLocation(44.33, 13.61, -66.78);
    public static CannonLocation cubeMidLocation = new CannonLocation(42.2, 7.8, -67.1);
    public static CannonLocation lowLocation = new CannonLocation(3.5, 0, -67);
    public static CannonLocation coneFloorPickLocation = new CannonLocation(-10.1, 0.0,5.0);
    public static CannonLocation chuteLocation = new CannonLocation(38.7, 0.82, -1.57);
    public static CannonLocation stationLocation = new CannonLocation(69.3, 8.89, -67);
    public static CannonLocation parkLocation = new CannonLocation(90, 1, -90);
    public static CannonLocation sidewaysConeLocation = new CannonLocation(-31.59, 0.0, 27.5);
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
