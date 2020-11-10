package frc.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class NewEightBallSim extends SequentialCommandGroup{
    public NewEightBallSim(DriveTrain robotDrive, DriveCommand drivecommand){
        drivecommand.cancel();
        robotDrive.resetOdometry(new Pose2d());
    var autoVoltageConstraint =
    new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(Constants.ksVolts,
                                   Constants.kvVoltSecondsPerMeter,
                                   Constants.kaVoltSecondsSquaredPerMeter),
        Constants.kDriveKinematics,
        10);
    TrajectoryConfig configForward =
        new TrajectoryConfig(Constants.kMaxSpeed,
                             Constants.kMaxAcceleration)
            .setKinematics(Constants.kDriveKinematics)
            .addConstraint(autoVoltageConstraint);
        
    TrajectoryConfig configBackward =
        new TrajectoryConfig(Constants.kMaxSpeed,
                             Constants.kMaxAcceleration)
            .setKinematics(Constants.kDriveKinematics)
            .addConstraint(autoVoltageConstraint)
            .setReversed(true);

    Trajectory ForwardTrajectory = TrajectoryGenerator.generateTrajectory(
        // Starting from Starting Point #2
        new Pose2d(0, 0, new Rotation2d(0)), 
        List.of(
            new Translation2d(-1.2, -0.63)
        ),
        new Pose2d(-5.5, -0.63, Rotation2d.fromDegrees(0)),
        configBackward
    ); 

    Trajectory BackTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(-6.5, -0.63, new Rotation2d(-10)), 
        List.of(
            //new Translation2d(-1.2, -0.63)
        ),
        new Pose2d(-4.2, -0.63, Rotation2d.fromDegrees(10)),
        configForward
    ); 

    }
}