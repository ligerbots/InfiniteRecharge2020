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
@SuppressWarnings("all")
public class TestFlup extends CommandBase {
  /**
   * Creates a new TestFlup.
   */

  Shooter shooter;
  public TestFlup(Shooter shooter) {
    this.shooter = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("Shooter P", 0.00017);
    SmartDashboard.putNumber("Shooter I", 0.00000);
    SmartDashboard.putNumber("Shooter D", 0);
    SmartDashboard.putNumber("Target Speed", -7000);

    shooter.setHood(70);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.calibratePID(SmartDashboard.getNumber("Shooter P", 0), SmartDashboard.getNumber("Shooter I", 0.00), SmartDashboard.getNumber("Shooter D", 0));
    shooter.shoot();
    //shooter.testSpin();
    System.out.println("Shooting");
    SmartDashboard.putNumber("Shooter RPM", shooter.getSpeed());
    System.out.println("Shooter speed: " + shooter.getSpeed());
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
