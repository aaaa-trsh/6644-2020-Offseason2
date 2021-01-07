package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.util.Units;
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
    
    public Command ramseteCommand(String trajectoryJSON) {
        TrajectoryConfig config = new TrajectoryConfig(
            Units.feetToMeters(2.0), Units.feetToMeters(2.0));
        config.setKinematics(drivebase.getKinematics());
        
        Path trajectoryPath = null;
        Trajectory trajectory = null;

        try {            
            trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
        }

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
        
        return command.andThen(() -> drivebase.tankDriveVolts(0, 0));
    }

    public Command getAutonomousCommand()
    {
        return new SequentialCommandGroup(
            ramseteCommand("paths/output/p1.wpilib.json"), 
            new InstantCommand(()->drivebase.tankDriveVolts(0, 0)), 
            new WaitCommand(2), 
            new InstantCommand(()->drivebase.setTransmission(true)), 
            new WaitCommand(1), 
            new InstantCommand(()->drivebase.setTransmission(false)), 
            new WaitCommand(2), 
            ramseteCommand("paths/output/p2.wpilib.json"),
            new InstantCommand(()->drivebase.tankDriveVolts(0, 0))
        );
    }

    public void reset()
    {
        drivebase.resetOdometry(new Pose2d());
    }
}
