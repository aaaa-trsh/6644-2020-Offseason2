package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
//import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase
{
    Encoder leftEncoder = new Encoder(DriveConstants.kLeftEncoderPorts[0], DriveConstants.kLeftEncoderPorts[1], true);
    Encoder rightEncoder = new Encoder(DriveConstants.kRightEncoderPorts[0], DriveConstants.kRightEncoderPorts[1], false);
    
    WPI_VictorSPX leftFront = new WPI_VictorSPX(DriveConstants.kLeftMotorPorts[0]);
    WPI_VictorSPX leftFollower = new WPI_VictorSPX(DriveConstants.kLeftMotorPorts[1]);
    WPI_VictorSPX rightFront = new WPI_VictorSPX(DriveConstants.kRightMotorPorts[0]);
    WPI_VictorSPX rightFollower = new WPI_VictorSPX(DriveConstants.kRightMotorPorts[1]);

    PIDController leftController = new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD);
    PIDController rightController = new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD);
    PIDController headingController = new PIDController(DriveConstants.kTurnP, DriveConstants.kTurnI, DriveConstants.kTurnD);

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(DriveConstants.kS, DriveConstants.kV, DriveConstants.kA);

    Gyro gyro = new ADXRS450_Gyro(Port.kOnboardCS0);

    DifferentialDrive differentialDrive = new DifferentialDrive(leftFront, rightFront);
    DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(DriveConstants.kTrackwidthMeters);
    DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getGyroHeading(), new Pose2d(0, 0, new Rotation2d()));
    
    Pose2d pose;

    Solenoid transmissionSolenoidL = new Solenoid(0);
    Solenoid transmissionSolenoidR = new Solenoid(1);

    //Field2d field;

    public DriveSubsystem() 
    {
        gyro.calibrate();
        leftFront.configFactoryDefault();
        leftFollower.configFactoryDefault();
        rightFront.configFactoryDefault();
        rightFollower.configFactoryDefault();

        leftFront.setInverted(InvertType.None);
        rightFront.setInverted(InvertType.InvertMotorOutput);

        leftFront.configOpenloopRamp(.5);
        leftFollower.configOpenloopRamp(.5);
        rightFront.configOpenloopRamp(.5);
        rightFollower.configOpenloopRamp(.5);

        leftFollower.follow(leftFront);
        rightFollower.follow(rightFront);

        leftFollower.setInverted(InvertType.FollowMaster);
        rightFront.setInverted(InvertType.FollowMaster);

        leftEncoder.reset();
        rightEncoder.reset();
        leftEncoder.setDistancePerPulse(DriveConstants.kDriveEncoderDPP);
        rightEncoder.setDistancePerPulse(DriveConstants.kDriveEncoderDPP);

        pose = odometry.getPoseMeters();

        SmartDashboard.putNumber("Rotation", getGyroHeading().getDegrees());
        SmartDashboard.putNumber("X", getPose().getTranslation().getX());
        SmartDashboard.putNumber("Y", getPose().getTranslation().getY());
        SmartDashboard.putNumber("left", getLeftEncoder().getDistance());
        SmartDashboard.putNumber("right", getRightEncoder().getDistance());
        //SmartDashboard.putData("Field", field);
    }

    public void setTransmission(final boolean on) {
        transmissionSolenoidL.set(on);
        transmissionSolenoidR.set(on);
    }

    public Rotation2d getGyroHeading() {
        return Rotation2d.fromDegrees(-gyro.getAngle());
    }

    public Encoder getLeftEncoder() {
        return leftEncoder;
    }

    public Encoder getRightEncoder() {
        return rightEncoder;
    }

    public DifferentialDriveWheelSpeeds getSpeeds() {
        return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
    }

    @Override
    public void periodic() {
        pose = odometry.update(getGyroHeading(), Units.feetToMeters(leftEncoder.getDistance()), Units.feetToMeters(rightEncoder.getDistance()));
        //field.setRobotPose(odometry.getPoseMeters());
    }

    public void tankDriveVolts(final double leftVolts, final double rightVolts) {
        differentialDrive.tankDrive(leftVolts / 12, rightVolts / 12);
    }

    public void arcadeDrive(final double forward, final double turn) {
        differentialDrive.arcadeDrive(-forward, -turn);
    }

    public DifferentialDriveKinematics getKinematics() {
        return kinematics;
    }

    public SimpleMotorFeedforward getFeedforward() {
        return feedforward;
    }

    public PIDController getLeftPIDController() {
        return leftController;
    }

    public PIDController getRightPIDController() {
        return rightController;
    }

    public Pose2d getPose() {
        return pose;
    }

    public void resetOdometry(final Pose2d pose2d) {
        leftEncoder.reset();
        rightEncoder.reset();
        gyro.reset();
        odometry.resetPosition(pose2d, getGyroHeading());
    }
}
