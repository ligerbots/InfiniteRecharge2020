/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Carousel;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;

public class TemporaryShooterCommand extends CommandBase {
  /**
   * Creates a new TemporaryShooterCommand.
   */

  CarouselCommand carouselCommand;
  Shooter shooter;
  Carousel carousel;
  DriveTrain robotDrive;
  int startTicks;

  long startTime;
  

  public TemporaryShooterCommand(Shooter shooter, Carousel carousel, DriveTrain robotDrive, CarouselCommand carouselCommand) {
    this.shooter = shooter;
    this.carousel = carousel;
    this.robotDrive = robotDrive;
    this.carouselCommand = carouselCommand;
    addRequirements(carousel);

    startTicks = carousel.getTicks();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    carouselCommand.cancel();
    shooter.calibratePID(0.00017, 0.00000013, 0);
    //shooter.testSpin();
    shooter.setHood(60);
    startTime = System.nanoTime();
    shooter.setLEDRing(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*if ((double)(System.nanoTime() - startTime) / 1_000_000_000.0 > 2.5);
      shooter.shoot();
    shooter.testSpin();
    carousel.spin(0.7);*/

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopAll();
    carouselCommand.schedule();
    shooter.setLEDRing(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (double)(System.nanoTime() - startTime) / 1_000_000_000 > 5; //startTicks - 62805 >= carousel.getTicks();
  }
}
