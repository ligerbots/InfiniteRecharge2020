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

public class FiveBallCenterAuto extends SequentialCommandGroup {
  /**
   * Add your docs here.
   */
  public FiveBallCenterAuto(final DriveTrain robotDrive, final Shooter shooter, final Intake intake, final Climber climber, final Carousel carousel, final DriveCommand driveCommand) {
    driveCommand.cancel();
    // intake.run(0.4);
    final IntakeCommand intakeCommand = new IntakeCommand(intake, climber, Constants.INTAKE_SPEED);
    robotDrive.resetOdometry(new Pose2d());
    final var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(Constants.ksVolts,
                                       Constants.kvVoltSecondsPerMeter,
                                       Constants.kaVoltSecondsSquaredPerMeter),
            Constants.kDriveKinematics,
            10);
    final TrajectoryConfig config =
      new TrajectoryConfig(Constants.kMaxSpeed,
          Constants.kMaxAcceleration)
          .setKinematics(Constants.kDriveKinematics)// Add kinematics to ensure max speed is actually obeyed
          .addConstraint(autoVoltageConstraint);// Apply the voltage constraint
    final Trajectory fiveBallTrajectory = TrajectoryGenerator.generateTrajectory(
      // Start at the origin facing the +X direction
      new Pose2d(0, 0, new Rotation2d(0)),
      List.of(
         new Translation2d(388.637 * Constants.inchToMetersConversionFactor, 173.480 * Constants.inchToMetersConversionFactor), // ball 7
         new Translation2d(394.975 * Constants.inchToMetersConversionFactor, 158.178 * Constants.inchToMetersConversionFactor), // ball 6                           
         new Translation2d(401.313 * Constants.inchToMetersConversionFactor, 142.877 * Constants.inchToMetersConversionFactor), // ball 5
         new Translation2d(394.200 * Constants.inchToMetersConversionFactor, 118.470 * Constants.inchToMetersConversionFactor), // ball 4
         new Translation2d(378.899 * Constants.inchToMetersConversionFactor, 112.131 * Constants.inchToMetersConversionFactor) // ball 3
      ),
      new Pose2d(122.1 * Constants.inchToMetersConversionFactor, 0, new Rotation2d(0)),
      config
      );


    final RamseteCommand ramseteCommand3 = new RamseteCommand(
        fiveBallTrajectory,
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

    addCommands(intakeCommand.alongWith(ramseteCommand3), andThen(() -> robotDrive.tankDriveVolts(0, 0)));//new StartMatchCommand(), new ShooterCommand (shooter, carousel, robotDrive, 3.0));
    
  }

}
