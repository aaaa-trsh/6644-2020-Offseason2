package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;

import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
    DriveSubsystem drivebase = new DriveSubsystem();
    Compressor compressor = new Compressor();
    Joystick joystick = new Joystick(0);

    NetworkTableEntry posEntry;

    public RobotContainer() {
        compressor.start();
        configureButtonBindings();

        NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
        NetworkTable table = ntInstance.getTable("testws");
        posEntry = table.getEntry("pos");
        
        // posEntry.setNumberArray(value)
    }

    private void configureButtonBindings() {
        drivebase.setDefaultCommand(new RunCommand(() -> drivebase.arcadeDrive(joystick.getY(), joystick.getX()), drivebase));
        JoystickButton button = new JoystickButton(joystick, 1);
        button.whenPressed(new InstantCommand(()->drivebase.setTransmission(true)))
            .whenReleased(new InstantCommand(()->drivebase.setTransmission(false)));
    }
    
    public void periodic() {
        posEntry.setDoubleArray(new double[]{ Units.metersToFeet(drivebase.getPose().getX()), Units.metersToFeet(drivebase.getPose().getY()), drivebase.getPose().getRotation().getRadians() });
    }

    public Command ramseteCommand(String trajectoryJSON) {
        TrajectoryConfig config = new TrajectoryConfig(.5, 10);
        config.setKinematics(drivebase.getKinematics());
        
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(
                new Translation2d(1, 1),
                new Translation2d(2, -1)
            ),
            new Pose2d(3, 0, new Rotation2d(0)),
            config
        );

        // try {
        //     Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
        //     trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        // } catch (IOException ex) {
        //     DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
        //     return null;
        // }
        
        drivebase.resetOdometry(trajectory.getInitialPose());

        RamseteCommand command = new RamseteCommand(
            trajectory,
            drivebase::getPose,
            new RamseteController(2, .7),
            drivebase.getFeedforward(),
            drivebase.getKinematics(),
            drivebase::getSpeeds,
            drivebase.getLeftPIDController(),
            drivebase.getRightPIDController(),
            drivebase::tankDriveVolts,
            drivebase
        );
    
        return command.andThen(() -> drivebase.tankDriveVolts(0, 0))
                      .andThen(
            new PIDCommand(
                new PIDController(
                    DriveConstants.kTurnP,
                    DriveConstants.kTurnI,
                    DriveConstants.kTurnD
                ),
                drivebase::getGyroAngle,
                0,
                (x) -> {
                    drivebase.arcadeDrive(0, x);
                },
                drivebase
            )
        );
    }

    public Command getAutonomousCommand()
    {
        return new SequentialCommandGroup(
            ramseteCommand("paths\\output\\CurveIn.wpilib.json")//, 
            // new InstantCommand(()->drivebase.setTransmission(true)), 
            // new WaitCommand(1), 
            // new InstantCommand(()->drivebase.setTransmission(false)), 
            // ramseteCommand("paths\\output\\CurveOut.wpilib.json")
        );
    }
}
