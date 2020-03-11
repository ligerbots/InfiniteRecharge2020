/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Carousel;

public class CarouselPIDCommand extends CommandBase {
  /**
   * Creates a new CarouselPIDCommand.
   */
  Carousel carousel;
  int currentError, nextTarget;
  int fifthRotationTicks = Constants.CAROUSEL_FIFTH_ROTATION_TICKS;
  double P = 0.0001;
  boolean stopForOpenSpace;

  public CarouselPIDCommand(Carousel carousel) {
    this.carousel = carousel;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    nextTarget = carousel.getTicks() + fifthRotationTicks;
    stopForOpenSpace = !carousel.isBallInFront();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentError = carousel.getTicks() - nextTarget;
    if (!stopForOpenSpace) {
      if (Math.abs(currentError) > 10) {
        carousel.spin(P * currentError);
      }
      else {
        stopForOpenSpace = carousel.isBallInFront();
        nextTarget += fifthRotationTicks;
      }
    }
    else {
      stopForOpenSpace = carousel.isBallInFront();
      carousel.spin(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
