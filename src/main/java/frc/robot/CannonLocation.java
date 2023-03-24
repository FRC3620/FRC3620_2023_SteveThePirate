// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


/** Add your docs here. */
public class CannonLocation {
    public static CannonLocation coneHighLocation = new CannonLocation(40.08, 29.79, -44.86);
    public static CannonLocation cubeHighLocation = new CannonLocation(40.21, 24.33, -55.93);
    public static CannonLocation coneMidLocation = new CannonLocation(42.59, 8.86, -46.91);
    public static CannonLocation cubeMidLocation = new CannonLocation(42.2, 7.8, -62.72);
    public static CannonLocation lowLocation = new CannonLocation(3.5, 0, -67);
    public static CannonLocation coneFloorPickLocation = new CannonLocation(-6.18, 0.0,-20.82);
    public static CannonLocation chuteLocation = new CannonLocation(28.65, 0.82, 10.86);
    public static CannonLocation stationLocation = new CannonLocation(73.14, 8.36, -79.06); //8.89
    public static CannonLocation parkLocation = new CannonLocation(90, 1, -130);

    public static CannonLocation sidewaysConeLocation = new CannonLocation(-26.5, 0.0, 35.05); //27.5
    public static CannonLocation backwardsHalfwayLocation = new CannonLocation(155, 0, 25);
    public static CannonLocation backwardsFloorPickupLocation = new CannonLocation(183.57, 0.0, 39.65); //elev: 190.4, pitch: 1.32
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
