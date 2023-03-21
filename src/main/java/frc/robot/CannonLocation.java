// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


/** Add your docs here. */
public class CannonLocation {
    public static CannonLocation coneHighLocation = new CannonLocation(43.45, 31.55, -44.86);
    public static CannonLocation cubeHighLocation = new CannonLocation(40.21, 24.33, -31.09);
    public static CannonLocation coneMidLocation = new CannonLocation(44.33, 10.72, -43.55);
    public static CannonLocation cubeMidLocation = new CannonLocation(42.2, 7.8, -51.38);
    public static CannonLocation lowLocation = new CannonLocation(3.5, 0, -67);
    public static CannonLocation coneFloorPickLocation = new CannonLocation(-5.95, 0.0,5.0);
    public static CannonLocation chuteLocation = new CannonLocation(38.7, 0.82, -1.57);
    public static CannonLocation stationLocation = new CannonLocation(69.3, 11.62, -67); //8.89
    public static CannonLocation parkLocation = new CannonLocation(90, 1, -130);
    public static CannonLocation sidewaysConeLocation = new CannonLocation(-17.37, 0.0, 27.5);
    public static CannonLocation backwardsHalfwayLocation = new CannonLocation(155, 0, 25);
    public static CannonLocation backwardsFloorPickupLocation = new CannonLocation(185.78, 0.0, 29.54);
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
