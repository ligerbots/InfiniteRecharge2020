/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Carousel;
import frc.robot.subsystems.Shooter;

public class ShootFromKey extends CommandBase {
  /**
   * Creates a new ShootFromKey.
   */
  Shooter shooter;
  Carousel carousel;
  CarouselCommand carouselCommand;
  int startTicks;

  public ShootFromKey(Shooter shooter, Carousel carousel, CarouselCommand carouselCommand) {
    this.shooter = shooter;
    this.carousel = carousel;
    this.carouselCommand = carouselCommand;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.calibratePID(0.000085, 0.000000033, 0, 6.776 * 0.00001);
    startTicks = carousel.getTicks();
    carouselCommand.cancel();
    shooter.setHood(155);
    shooter.setShooterRPM(-4000);
    shooter.setTurretAdjusted(0.1);
    shooter.shoot();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shooter.getSpeed() < -3950) {
      carousel.spin(0.5);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopAll();
    carouselCommand.schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return startTicks - 62805 >= carousel.getTicks();
  }
}