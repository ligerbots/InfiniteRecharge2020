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
import edu.wpi.first.wpilibj.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.FieldMap;
import frc.robot.subsystems.Carousel;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;

public class TrenchAuto extends SequentialCommandGroup implements AutoCommandInterface {

    // Define the initial pose to be used by this command. This will be used in the initial trajectory
    // and will allow the system to query for it
    private Pose2d initialPose;

    public TrenchAuto(Pose2d initialPose, DriveTrain robotDrive, DriveCommand drivecommand, Climber climber, Carousel carousel, CarouselCommand carouselcommand, Shooter shooter) {
        drivecommand.cancel();

        // Save the passed in initialPose so we can use it later
        this.initialPose = initialPose;

        DeployShoulderCommand deployShoulder = new DeployShoulderCommand(climber);
        var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(new SimpleMotorFeedforward(Constants.ksVolts,
                Constants.kvVoltSecondsPerMeter, Constants.kaVoltSecondsSquaredPerMeter), Constants.kDriveKinematics,
                10);
        TrajectoryConfig configForward = new TrajectoryConfig(Constants.kMaxSpeed, Constants.kMaxAcceleration)
                .setKinematics(Constants.kDriveKinematics).addConstraint(autoVoltageConstraint);

        TrajectoryConfig configBackward = new TrajectoryConfig(Constants.kMaxSpeed, Constants.kMaxAcceleration)
                .setKinematics(Constants.kDriveKinematics).addConstraint(autoVoltageConstraint).setReversed(true);


        //System.out.println("DEBUG: " + FieldMap.ballPosition[5] + " " + FieldMap.ballPosition[4]);

        Trajectory backTrajectory = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                List.of( 
                    initialPose,
                    new Pose2d(FieldMap.ballPosition[7], Rotation2d.fromDegrees(0)), 
                    new Pose2d(FieldMap.ballPosition[10].getX(), (FieldMap.ballPosition[10].getY() + FieldMap.ballPosition[11].getY())/2,Rotation2d.fromDegrees(0))
                    
                ),
                configBackward);
        Trajectory forwardTrajectory = TrajectoryGenerator.generateTrajectory(
        // Starting from Starting Point #2
                new Pose2d(FieldMap.ballPosition[10].getX(), (FieldMap.ballPosition[10].getY() + FieldMap.ballPosition[11].getY())/2,Rotation2d.fromDegrees(0)),
                List.of(
                    FieldMap.ballPosition[8]
                ),
                new Pose2d(FieldMap.ballPosition[7].plus(new Translation2d(.4,.4)), Rotation2d.fromDegrees(0)),
                configForward);   

        // for (State state : backTrajectory.getStates()) {
        //     System.out.println("DEBUG: backTrajectory STATE "+ state.poseMeters);
        // }
        // for (State state : forwardTrajectory.getStates()) {
        //     System.out.println("DEBUG: forwardTrajectory STATE "+ state.poseMeters);
        // }

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
        addCommands(
            //new SetTrajectory(robotDrive, configBackward).andThen(() -> robotDrive.tankDriveVolts(0, 0))//,
            ramseteBackward.andThen(() -> robotDrive.tankDriveVolts(0, 0)),
            ramseteForward.andThen(() -> robotDrive.tankDriveVolts(0, 0)),
            new TurnAndShoot(robotDrive, shooter, carousel, carouselcommand, drivecommand,false)
        );
    }

    // Allows the system to get the initial pose of this command
    public Pose2d getInitialPose() {
        return initialPose;
    }
}