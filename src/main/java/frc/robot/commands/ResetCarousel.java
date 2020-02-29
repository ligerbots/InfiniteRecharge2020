/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Carousel;

public class ResetCarousel extends CommandBase {
  /**
   * Creates a new ResetCarousel.
   */
  int currentCheckpoint;
  int startCheckpoint;
  int currentTicks;

  Carousel carousel;
  final int fifthRotationTicks = 12561;
  boolean reschedule;

  CarouselCommand carouselCommand;


  public ResetCarousel(Carousel carousel, CarouselCommand carouselCommand, boolean reschedule) {
    this.carousel = carousel;
    this.reschedule = reschedule;
    this.carouselCommand = carouselCommand;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startCheckpoint = -carousel.getTicks() / fifthRotationTicks;
    carousel.spin(0.5);
    carouselCommand.cancel();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    currentTicks = -carousel.getTicks();
    currentCheckpoint = currentTicks / fifthRotationTicks;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    carousel.spin(0);
    if (reschedule) {
      carouselCommand.schedule();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return currentCheckpoint > startCheckpoint;
  }
}
