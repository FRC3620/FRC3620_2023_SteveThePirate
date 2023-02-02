package org.usfirst.frc3620.misc;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;

abstract public class PoseOnField {
    public enum UnitsOfLength { INCH, METER };
    final static double inchesPerMeter = Units.metersToInches(1);

    final static Translation2d t00 = new Translation2d(0, 0);
    Translation2d translationInMeters = t00;
    Rotation2d rotation = null;

    PoseOnField(Translation2d where, UnitsOfLength uol, Double angleInDegrees) {
        if (uol == null) uol = UnitsOfLength.INCH;
        switch (uol) {
            case INCH:
                translationInMeters = where.div(inchesPerMeter);
                break;
            case METER:
                translationInMeters = where;
                break;
            default:
                throw new IllegalArgumentException("Unsupported enum value");
        }
        if (angleInDegrees != null) {
            // TODO hmmmmm.
        }
    }

    public Translation2d getTranslationInMeters() {
        return getTranslationInMeters(DriverStation.getAlliance());
    }

    abstract public Translation2d getTranslationInMeters(Alliance alliance);

    public Translation2d getTranslationInInches() {
        return getTranslationInInches(DriverStation.getAlliance());
    }

    public Translation2d getTranslationInInches(Alliance alliance) {
        return getTranslationInMeters(alliance).times(inchesPerMeter);
    }

    static class PoseOnFieldSetFromCenterLine extends PoseOnField {
        public PoseOnFieldSetFromCenterLine(Translation2d where, UnitsOfLength uol, Double angleInDegrees) {
            super(where, uol, angleInDegrees);
        }

        @Override
        public Translation2d getTranslationInMeters(Alliance alliance) {
            double halfFieldX = Constants.FIELD_LENGTH_IN_METERS / 2;
            double x;
            switch (alliance) {
                case Red:
                    x = halfFieldX + translationInMeters.getX();
                    break;
                case Blue:
                    x = halfFieldX - translationInMeters.getX();
                    break;
                default:
                    throw new IllegalArgumentException("Unsupported enum value");
            }
            return new Translation2d(x, translationInMeters.getY());
        }
    }

    public static PoseOnField makePoseFromCenterLineInInches (Translation2d t) {
        return new PoseOnFieldSetFromCenterLine(t, UnitsOfLength.INCH, null);
    }

    public static PoseOnField makePoseFromCenterLineInMeters (Translation2d t) {
        return new PoseOnFieldSetFromCenterLine(t, UnitsOfLength.METER, null);
    }

    static class PoseOnFieldSetFromMyWall extends PoseOnField {
        public PoseOnFieldSetFromMyWall(Translation2d where, UnitsOfLength uol, Double angleInDegrees) {
            super(where, uol, angleInDegrees);
        }

        @Override
        public Translation2d getTranslationInMeters(Alliance alliance) {
            double fieldX = Constants.FIELD_LENGTH_IN_METERS;
            double x;
            switch (alliance) {
                case Red:
                    x = fieldX - translationInMeters.getX();
                    break;
                case Blue:
                    x = translationInMeters.getX();
                    break;
                default:
                    throw new IllegalArgumentException("Unsupported enum value");
            }
            return new Translation2d(x, translationInMeters.getY());
        }
    }

    public static PoseOnField makePoseFromMyWallInInches (Translation2d t) {
        return new PoseOnFieldSetFromMyWall(t, UnitsOfLength.INCH, null);
    }

    public static PoseOnField makePoseFromMyWallInMeters (Translation2d t) {
        return new PoseOnFieldSetFromMyWall(t, UnitsOfLength.METER, null);
    }

}
