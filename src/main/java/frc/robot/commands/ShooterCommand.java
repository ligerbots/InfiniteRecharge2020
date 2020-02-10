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

  Shooter shooter;
  Carousel carousel;
  DriveTrain robotDrive;

  boolean startShooting;

  public ShooterCommand(Shooter shooter, Carousel carousel, DriveTrain robotDrive) {
    this.shooter = shooter;
    this.carousel = carousel;
    this.robotDrive = robotDrive;

    startShooting = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //TODO: remember to set to shooting camera mode!!

  }

  // Called every time the scheduler runs while the command is scheduled.
  double angleError;
  double distance;

  boolean speedOnTarget = false;
  boolean hoodOnTarget = false;
  boolean angleOnTarget = false;

  @Override
  public void execute() {
    visionInfo = SmartDashboard.getNumberArray("visionInfo", empty); // TODO: need actual vision info

    if (visionInfo[0] != 0) { // figure out if we see a vision target
        shooter.prepareShooter(visionInfo[2]);

        angleError = visionInfo[3];
        distance = visionInfo[2];

        if (angleError > Constants.MAX_TURRET_OFFSET) {
          robotDrive.allDrive(0, 0.4 * Math.signum(angleError));
        }
        else {
          robotDrive.allDrive(0, 0);
          shooter.setTurret(angleError *  Math.signum(angleError));
        }

        speedOnTarget = shooter.speedOnTarget(shooter.calculateShooterSpeed(visionInfo[2]), 1); //TODO: May need to adjust acceptable error
        hoodOnTarget = shooter.hoodOnTarget(shooter.calculateShooterHood(visionInfo[2]));
        angleOnTarget = Math.abs(shooter.getTurretAngle() + angleError) <= 1.5; // They should be opposites so I added them



        if (speedOnTarget && hoodOnTarget && angleOnTarget)
            shooter.shoot();
    }

  }

  // if (shooter.speedOnTarget(shooter.calculateShooterSpeed(visionInfo[1]), 1) && shooter.hoodOnTarget(shooter.calculateShooterHood(visionInfo[1]))) {
  //   shooter.shoot();
  // } //The allowed error here matters a lot

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopAll();
    carousel.spin(Constants.CAROUSEL_INTAKE_SPEED);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
