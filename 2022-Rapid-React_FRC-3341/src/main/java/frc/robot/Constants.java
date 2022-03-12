// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class Field {
        public static final double HubHeightMeters = 2.64; // 8.667 Feet
        public static final double BufferHeightMeters = 0.5;
    }

    public static final class BallHandler {

        public static final class Pivot {
            public static final int GearRatio = 1;
            public static final int EncoderDeadband = 5;
            public static final double PivotPower = 0.2;
            public static final double FeedForwardPower = 0.5;
            public static final double StowedEncoderPosition = 0.0;
            public static final double ScanEncoderPosition = 683.0;
            public static final double IntakeEncoderPosition = 1536;
        }

        public static final class Flywheel {
            public static final double FlywheelRadiusMeters = 0.0508;
            public static final double VelocityDeadband = 1;
            public static final double IntakePower = -0.2;
                // TBD TBD: Change intake speed to RPM or linear velocity
        }

        public static final class Collector {
            public static final int CollectorRadiusMeters = 1;
            public static final double IntakePower = -0.5;
        }

        public static final class Measurements {
            public static final double LimelightToPivotpointMeters = 1;
            public static final double LimelightToPivotpointDegrees = 45;
            public static final double FlywheelsToPivotpointMeters = 1;
            public static final double PivotpointToFloorMeters = 1;
        }
    }

    public static final class DriveTrain {
        public static final double TurnSpeed = 0.8;
    }

    public static final class Values {
        public static final int EncoderTicksPerRotation = 4096;
        public static final int DegreesPerRotation = 360;
        public static final int MillisecondsPerSecond = 1000;
    }

    public static final class CanID {
        public static final class DriveTrain {
            public static final int LeftDriveTalon = 2;
            public static final int RightDriveTalon = 3;
            public static final int LeftFollowerVictor = 4;
            public static final int RightFollowerVictor = 5;
        }

        public static final class BallHandler {
            public static final int PivotTalon = 14;
            public static final int FeederTalon = 15;
            public static final int LeftFlywheel = 16;
            public static final int RightFlywheel = 17;
        }
    }

    public static final class USBOrder {
        public static final int ZERO = 0;
        public static final int ONE = 1;
    }

    public static final class JoystickAxis {
        public static final int XAxis = 0;
        public static final int YAxis = 1;
    }
}
