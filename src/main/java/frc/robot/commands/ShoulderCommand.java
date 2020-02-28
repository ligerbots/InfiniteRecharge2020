/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Climber;

public class ShoulderCommand extends CommandBase {
  /**
   * Creates a new ShoulderCommand.
  */

  Climber climber;
  double requestedShoulderHeight;
  
  public ShoulderCommand(Climber climber, double shoulderHeight) {
    this.climber = climber;
    requestedShoulderHeight = shoulderHeight;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climber.shoulder.setIdleMode(IdleMode.kCoast);
    climber.moveShoulder(requestedShoulderHeight);
    SmartDashboard.putNumber("Shoulder set point", requestedShoulderHeight);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (climber.getShoulderPosition() >= requestedShoulderHeight - 2.0/360.0);
  }
}
