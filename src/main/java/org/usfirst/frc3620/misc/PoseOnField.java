package org.usfirst.frc3620.misc;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;

abstract public class PoseOnField {
    public enum UnitsOfLength { INCH, METER };
    final static double inchesPerMeter = Units.metersToInches(1);

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

    abstract static class PoseOnFieldBase extends PoseOnField {

        final Translation2d translationInMeters;
        final Rotation2d rotation = null;

        PoseOnFieldBase(Translation2d where, UnitsOfLength uol, Double angleInDegrees) {
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

    }

    static class PoseOnFieldSetFromCenterLine extends PoseOnFieldBase {
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

    public static PoseOnField fromCenterLineInInches (Translation2d t) {
        return new PoseOnFieldSetFromCenterLine(t, UnitsOfLength.INCH, null);
    }

    public static PoseOnField fromCenterLineInMeters (Translation2d t) {
        return new PoseOnFieldSetFromCenterLine(t, UnitsOfLength.METER, null);
    }

    static class PoseOnFieldSetFromMyWall extends PoseOnFieldBase {
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

    public static PoseOnField fromMyWallInInches (Translation2d t) {
        return new PoseOnFieldSetFromMyWall(t, UnitsOfLength.INCH, null);
    }

    public static PoseOnField fromMyWallInMeters (Translation2d t) {
        return new PoseOnFieldSetFromMyWall(t, UnitsOfLength.METER, null);
    }

    static class PoseOnFieldSetFromRedAlliance extends PoseOnFieldBase {
        public PoseOnFieldSetFromRedAlliance(Translation2d where, UnitsOfLength uol, Double angleInDegrees) {
            super(where, uol, angleInDegrees);
        }

        @Override
        public Translation2d getTranslationInMeters(Alliance alliance) {
            double x;
            switch (alliance) {
                case Red:
                    x = translationInMeters.getX();
                    break;
                case Blue:
                    x = Constants.FIELD_LENGTH_IN_METERS - translationInMeters.getX();
                    break;
                default:
                    throw new IllegalArgumentException("Unsupported enum value");
            }
            return new Translation2d(x, translationInMeters.getY());
        }
    }

    public static PoseOnField fromRedAlliancePositionInMeters(double x, double y) {
        Translation2d t = new Translation2d(x, y);
        return new PoseOnFieldSetFromRedAlliance(t, UnitsOfLength.METER, null);
    }

    public static PoseOnField fromRedAlliancePositionInMeters(Translation2d t) {
        return new PoseOnFieldSetFromRedAlliance(t, UnitsOfLength.METER, null);
    }

}
