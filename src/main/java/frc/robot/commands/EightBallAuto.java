/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

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
import frc.robot.subsystems.Carousel;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class EightBallAuto extends SequentialCommandGroup {
  /**
   * Add your docs here.
   */
  public EightBallAuto(DriveTrain robotDrive, Shooter shooter, Intake intake, Climber climber, Carousel carousel, DriveCommand driveCommand, CarouselCommand carouselCommand) {
    driveCommand.cancel();
    intake.run(0.4);
    IntakeCommand intakeCommand = new IntakeCommand(intake, climber, 0.5);
    TurnAndShoot shoot1 = new TurnAndShoot(robotDrive, shooter, carousel, carouselCommand, driveCommand, false);
    TurnAndShoot shoot2 = new TurnAndShoot(robotDrive, shooter, carousel, carouselCommand, driveCommand, false);
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

    Trajectory temporaryTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)), 
        List.of(
            new Translation2d(-1.2, -0.63)
        ),
        new Pose2d(-5.5, -0.63, Rotation2d.fromDegrees(0)),
        configBackward
    ); 

    Trajectory comeBackTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(-6.5, -0.63, new Rotation2d(-10)), 
        List.of(
            //new Translation2d(-1.2, -0.63)
        ),
        new Pose2d(-4.2, -0.63, Rotation2d.fromDegrees(10)),
        configForward
    ); 


    RamseteCommand ramseteCommand1 = new RamseteCommand(
        temporaryTrajectory,
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

    RamseteCommand ramseteCommand2 = new RamseteCommand(
        comeBackTrajectory,
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


    addCommands(deployShoulder.alongWith(shoot1),
        new SetTrajectory(robotDrive, configBackward).andThen(() -> robotDrive.tankDriveVolts(0, 0)),
        ramseteCommand2.andThen(() -> robotDrive.tankDriveVolts(0, 0)),
        new ResetCarousel(carousel, carouselCommand, false), 
        new TurnAndShoot(robotDrive, shooter, carousel, carouselCommand, driveCommand, false), 
        new ResetCarousel(carousel, carouselCommand, true)
    );// new StartMatchCommand(), new
                                                                                 // ShooterCommand (shooter, carousel,
                                                                                 // robotDrive, 3.0));
    
  }


}
