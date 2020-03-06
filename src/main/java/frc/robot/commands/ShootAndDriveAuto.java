/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Carousel;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ShootAndDriveAuto extends SequentialCommandGroup {
  /**
   * Creates a new ShootAndDriveAuto.
   */
  public ShootAndDriveAuto(DriveTrain robotDrive, Shooter shooter, Intake intake, Climber climber, Carousel carousel, DriveCommand driveCommand, CarouselCommand carouselCommand) {

    TurnAndShoot shoot1 = new TurnAndShoot(robotDrive, shooter, carousel, carouselCommand, driveCommand, false);
    DeployShoulderCommand deployShoulder = new DeployShoulderCommand(climber);

    addCommands(deployShoulder.alongWith(shoot1), new DriveForwardAuto(robotDrive, carouselCommand, driveCommand));
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());super();
  }
}
