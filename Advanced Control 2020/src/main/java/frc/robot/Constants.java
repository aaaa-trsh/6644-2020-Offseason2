package frc.robot;

import edu.wpi.first.wpilibj.util.Units;

public final class Constants {
    public static class DriveConstants
    {
        public static int[] kLeftMotorPorts = {0, 1};
        public static int[] kRightMotorPorts = {2, 3};

        public static int[] kLeftEncoderPorts = {0, 1};
        public static int[] kRightEncoderPorts = {2, 3};

        public static double kS = 0.977;
        public static double kV = 2.28;
        public static double kA = 0.0569;

        public static double kP = 1.1;
        public static double kI = 0;
        public static double kD = 0;

        public static double kTurnP = 0;
        public static double kTurnI = 0;
        public static double kTurnD = 0;

        public static int kDriveEncoderResolution = 360;
        public static double kWheelDiameterMeters = Units.inchesToMeters(6); // inches
        public static double kDriveEncoderDPP = (kWheelDiameterMeters * Math.PI)/kDriveEncoderResolution;

        public static double kMaxVelocity = 3; // m / s
        public static double kMaxAcceleration = 3; // m / s^2

        public static double kTrackwidthMeters = 0.6397603419838763; // 2.0989512532279404 feet
    }

    public static class OIConstants
    {
        public static int kDriveControllerPort = 0;
    }
}
