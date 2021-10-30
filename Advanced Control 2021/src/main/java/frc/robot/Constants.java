package frc.robot;

public final class Constants {
    public static class DriveConstants
    {
        public static int[] kLeftMotorPorts = {0, 1};
        public static int[] kRightMotorPorts = {2, 3};

        public static int[] kLeftEncoderPorts = {0, 1};
        public static int[] kRightEncoderPorts = {2, 3};

        public static double kS = 0.99;
        public static double kV = 4.31;
        public static double kA = 2.443;

        public static double kP = 20;
        public static double kI = .2;
        public static double kD = 0.3;

        public static double kTurnP = .02;
        public static double kTurnI = 0.0;
        public static double kTurnD = 0.003;

        public static int kDriveEncoderResolution = 360;
        public static double kWheelDiameter = 6; // inches
        public static double kDriveEncoderDPP = (kWheelDiameter * Math.PI)/kDriveEncoderResolution/12;

        public static double kMaxVelocity = 3; // feet / s
        public static double kMaxAcceleration = 3; // feet / s^2

        public static double kTrackwidthMeters = 0.6397603419838763; // 2.0989512532279404 feet
    }

    public static class OIConstants
    {
        public static int kDriveControllerPort = 0;
    }
}
