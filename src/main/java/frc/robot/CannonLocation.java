// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


/** Add your docs here. */
public class CannonLocation {
    public static CannonLocation coneHighLocation = new CannonLocation(44.84, 31.40, -51.30);
    public static CannonLocation halfwayToConeHighLocation = new CannonLocation(50, 31.395, -53.69);
    public static CannonLocation cubeHighLocation = new CannonLocation(43.10, 24.33, -47.24);
    public static CannonLocation coneMidLocation = new CannonLocation(46.29, 12.21, -61.31);
    public static CannonLocation cubeMidLocation = new CannonLocation(41.75, 5.6, -51.38);
    public static CannonLocation lowLocation = new CannonLocation(3.5, 0, -67);
    public static CannonLocation cubePickLocation = new CannonLocation(-14.43, 0.0,.26); //-11.56, 0.0, 0.26
    public static CannonLocation chuteLocation = new CannonLocation(28.65, 0.82, 10.86);
    public static CannonLocation stationLocation = new CannonLocation(70.71, 10.39, -77.9); //8.89
    public static CannonLocation parkLocation = new CannonLocation(90, 1, -130);

    public static CannonLocation coneFloorPickLocation = new CannonLocation(-21.506, 6.451, 37.573); //27.5
    public static CannonLocation backwardsHalfwayLocation = new CannonLocation(175, 0, 25);
    public static CannonLocation backwardsFloorPickupLocation = new CannonLocation(180.835, 0.0, 43.88); //elev: 183.22, pitch: 39.29

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
