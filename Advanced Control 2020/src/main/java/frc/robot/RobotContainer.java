package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
    DriveSubsystem drivebase = new DriveSubsystem();
    Compressor compressor = new Compressor();
    Joystick joystick = new Joystick(0);

    public RobotContainer() {
        compressor.start();
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        drivebase.setDefaultCommand(new RunCommand(() -> drivebase.arcadeDrive(joystick.getY(), joystick.getX()), drivebase));
        JoystickButton button = new JoystickButton(joystick, 1);
        button.whenPressed(new InstantCommand(()->drivebase.setTransmission(true)))
            .whenReleased(new InstantCommand(()->drivebase.setTransmission(false)));
    }
    
    public Command ramseteCommand(String trajectoryJSON) 
    {
        /*DifferentialDriveVoltageConstraint autoVoltageConstraint = 
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                DriveConstants.kS,
                DriveConstants.kV,
                DriveConstants.kA),
            drivebase.getKinematics(),
            10);
            
        TrajectoryConfig config = new TrajectoryConfig(
            Units.feetToMeters(5.0), Units.feetToMeters(5.0));
        config.setKinematics(drivebase.getKinematics());
        config.addConstraint(autoVoltageConstraint);
        
        
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(
                new Translation2d(1, 2),
                new Translation2d(2, -2)
            ),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            // Pass config
            config
        );*/

        Path trajectoryPath = null;
        Trajectory trajectory = null;
        try {            
            trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
        }

        trajectory = trajectory.transformBy(drivebase.getPose().minus(trajectory.getInitialPose()));

        RamseteCommand command = new RamseteCommand(
            trajectory,
            drivebase::getPose,
            new RamseteController(2, .7),
            new SimpleMotorFeedforward(
                DriveConstants.kS,
                DriveConstants.kV,
                DriveConstants.kA),
            drivebase.getKinematics(),
            drivebase::getSpeeds,
            drivebase.getLeftPIDController(),
            drivebase.getRightPIDController(),
            drivebase::tankDriveVolts,
            drivebase
        );
        
        return command.andThen(() -> drivebase.tankDriveVolts(0, 0));
    }

    public Command getAutonomousCommand()
    {
        return new SequentialCommandGroup(
            new InstantCommand(()->drivebase.setTransmission(false)), 
            ramseteCommand("paths/AutoNav_Barrel.wpilib.json"), 
            new InstantCommand(()->drivebase.tankDriveVolts(0, 0)), 
            new InstantCommand(()->drivebase.setTransmission(true)), 
            new WaitCommand(.2), 
            new InstantCommand(()->drivebase.setTransmission(false)), 
            new InstantCommand(()->drivebase.tankDriveVolts(0, 0))
        );
    }

    public void resetOdometry()
    {
        drivebase.resetGyro();
        drivebase.resetOdometry(new Pose2d());
    }
}
