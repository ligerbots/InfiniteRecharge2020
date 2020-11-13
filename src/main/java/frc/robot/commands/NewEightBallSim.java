package frc.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.FieldMap;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;

public class NewEightBallSim extends SequentialCommandGroup{
    public NewEightBallSim(DriveTrain robotDrive, DriveCommand drivecommand, Climber climber){
        drivecommand.cancel();
        DeployShoulderCommand deployShoulder = new DeployShoulderCommand(climber);
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

    Trajectory forwardTrajectory = TrajectoryGenerator.generateTrajectory(
        // Starting from Starting Point #2
        List.of(
            new Pose2d(FieldMap.ballPosition[3], Rotation2d.fromDegrees(67.5)),
            new Pose2d(FieldMap.startLineX, FieldMap.startPositions[2].getY(), Rotation2d.fromDegrees(0))
        ),
        configBackward
    ); 

    Trajectory backTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        FieldMap.startPositions[2], 
        List.of(
            FieldMap.ballPosition[5],
            FieldMap.ballPosition[4]
        ),
        new Pose2d(FieldMap.ballPosition[3], Rotation2d.fromDegrees(67.5)),
        configForward
    ); 
    
    RamseteCommand ramseteBackward = new RamseteCommand(
        backTrajectory,
        robotDrive::getPose,
        new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
        new SimpleMotorFeedforward(Constants.ksVolts,
                                   Constants.kvVoltSecondsPerMeter,
                                   Constants.kaVoltSecondsSquaredPerMeter),
        Constants.kDriveKinematics,
        robotDrive::getWheelSpeeds,
        new PIDController(Constants.kPDriveVel, 0, 0),
        new PIDController(Constants.kPDriveVel, 0, 0),
        robotDrive::tankDriveVolts,
        robotDrive
    );
    RamseteCommand ramseteForward = new RamseteCommand(
        forwardTrajectory,
        robotDrive::getPose,
        new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
        new SimpleMotorFeedforward(Constants.ksVolts,
                                   Constants.kvVoltSecondsPerMeter,
                                   Constants.kaVoltSecondsSquaredPerMeter),
        Constants.kDriveKinematics,
        robotDrive::getWheelSpeeds,
        new PIDController(Constants.kPDriveVel, 0, 0),
        new PIDController(Constants.kPDriveVel, 0, 0),
        robotDrive::tankDriveVolts,
        robotDrive
    );
    addCommands(deployShoulder,
    new SetTrajectory(robotDrive, configBackward).andThen(() -> robotDrive.tankDriveVolts(0, 0)),
    ramseteBackward.andThen(() -> robotDrive.tankDriveVolts(0, 0)),
    ramseteForward.andThen(() -> robotDrive.tankDriveVolts(0, 0)));
    }
}