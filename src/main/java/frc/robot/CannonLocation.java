// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


/** Add your docs here. */
public class CannonLocation {
    public static CannonLocation coneHighLocation = new CannonLocation(41.97, 31.395, -53.69);
    public static CannonLocation halfwayToConeHighLocation = new CannonLocation(50, 31.395, -53.69);
    public static CannonLocation cubeHighLocation = new CannonLocation(43.10, 24.33, -47.24);
    public static CannonLocation coneMidLocation = new CannonLocation(40.04, 10.55, -46.91);
    public static CannonLocation cubeMidLocation = new CannonLocation(36.78, 5.6, -51.38);
    public static CannonLocation lowLocation = new CannonLocation(3.5, 0, -67);
    public static CannonLocation coneFloorPickLocation = new CannonLocation(-3.68, 0.0,-30.52);
    public static CannonLocation chuteLocation = new CannonLocation(28.65, 0.82, 10.86);
    public static CannonLocation stationLocation = new CannonLocation(72.62, 7.7, -77.9); //8.89
    public static CannonLocation parkLocation = new CannonLocation(90, 1, -130);

    public static CannonLocation sidewaysConeLocation = new CannonLocation(-27.99, 0.0, 33.76); //27.5
    public static CannonLocation backwardsHalfwayLocation = new CannonLocation(160, 0, 25);
    public static CannonLocation backwardsFloorPickupLocation = new CannonLocation(185.24, 0.0, 39.29); //elev: 190.4, pitch: 1.32

    public static CannonLocation cubeHigherLocation = new CannonLocation(46.45, 24.33, -47.24);
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
