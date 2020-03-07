/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class ShooterTuner extends CommandBase {
  /**
   * Creates a new ShooterTuner.
   */

  Shooter shooter;
  public ShooterTuner(Shooter shooter) {
    this.shooter = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.calibratePID(0.000145, 1e-8, 0, 6.6774 * 0.00001);
    shooter.setLEDRing(true);
    SmartDashboard.putString("vision/active_mode/selected", "goalfinder");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Distance", SmartDashboard.getNumberArray("vision/target_info", new double[]{0,0,0,0,0,0,0})[3]);
    System.out.println("Shooter Tuner going!");
    shooter.shoot();
    shooter.setHood(SmartDashboard.getNumber("Target Hood Angle", 60));
    shooter.setShooterRPM(SmartDashboard.getNumber("TSR", -1000));
    shooter.setTurret(SmartDashboard.getNumber("Turret Angle", 72));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
