/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Carousel;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;

public class ShooterCommand extends CommandBase {
  /**
   * Creates a new ShooterCommand.
   */

  double[] visionInfo;
  double[] empty = new double[] {0.0,0.0,0.0,0.0,0.0,0.0,0.0}; 
  double waitTime;
  double startTime;

  Shooter shooter;
  Carousel carousel;
  DriveTrain robotDrive;

  boolean startShooting;

  CarouselCommand carouselCommand;

  int initialCarouselTicks;

  public ShooterCommand(Shooter shooter, Carousel carousel, DriveTrain robotDrive, double waitTime, CarouselCommand carouselCommand) {
    this.shooter = shooter;
    this.carousel = carousel;
    this.robotDrive = robotDrive;
    this.waitTime = waitTime;
    this.carouselCommand = carouselCommand;
    startShooting = false;
  }

  public void rapidFire() {
    carousel.spin(0.5);
    shooter.shoot();
}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = System.nanoTime();
    SmartDashboard.putString("vision/active_mode/selected", "goalfinder");
    shooter.setLEDRing(true);
    //TODO: remember to set to shooting camera mode!!
    carouselCommand.cancel();

    // stor current carouselTick value
    initialCarouselTicks = carousel.getTicks();
  }

  // Called every time the scheduler runs while the command is scheduled.
  double angleError;
  double distance;

  boolean speedOnTarget = false;
  boolean hoodOnTarget = false;
  boolean angleOnTarget = false;

  @Override
  public void execute() {
    visionInfo = SmartDashboard.getNumberArray("vision/target_info", empty); // TODO: need actual vision info

    if (visionInfo[0] != 0) { // figure out if we see a vision target
        angleError = visionInfo[4];
        distance = visionInfo[3];

        shooter.prepareShooter(distance);

        if (Math.abs(angleError) > 5) {
          robotDrive.allDrive(0, robotDrive.turnSpeedCalc(angleError), false);
        }
        else {
          robotDrive.allDrive(0, 0, false);
          //shooter.setTurret(angleError *  Math.signum(angleError));
        }

        speedOnTarget = shooter.speedOnTarget(shooter.calculateShooterSpeed(distance), 5); //TODO: May need to adjust acceptable error
        hoodOnTarget = (double)(System.nanoTime() - startTime) / 1_000_000_000 > 0.75;//shooter.hoodOnTarget(shooter.calculateShooterHood(distance));
        angleOnTarget = Math.abs(shooter.getTurretAngle() + angleError) <= 1.5; // They should be opposites so I added them

        if (speedOnTarget && hoodOnTarget/* && angleOnTarget*/)
            rapidFire();
    }

  }

  // if (shooter.speedOnTarget(shooter.calculateShooterSpeed(visionInfo[1]), 1) && shooter.hoodOnTarget(shooter.calculateShooterHood(visionInfo[1]))) {
  //   shooter.shoot();
  // } //The allowed error here matters a lot

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopAll();
    shooter.setLEDRing(false);
    carousel.setBallCount(0);
    carouselCommand.schedule();
  }

  // Returns true when the command should end.



  @Override
  public boolean isFinished() {
    // TODO: this should just check to see if the carousel has rotated 5 CAROUSEL_FIFTH_ROTATION_TICKS intervals
    return (carousel.getTicks() -initialCarouselTicks) > 5 * Constants.CAROUSEL_FIFTH_ROTATION_TICKS;
    // if (waitTime == 0.0) {
    //   return false;
    // }
    // else {
    //   return ((System.nanoTime() - startTime) / 1_000_000_000.0 >= waitTime);
    // }
  }
}
