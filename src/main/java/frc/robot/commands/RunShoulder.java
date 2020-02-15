/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;
import frc.robot.Robot;
import java.util.function.DoubleSupplier;
import frc.robot.subsystems.Winch;
import edu.wpi.first.wpilibj2.command.CommandBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
public class RunShoulder extends CommandBase {

  DoubleSupplier climb;
  public static CANSparkMax shoulder;
  /**
   * Creates a new RunClimber.
   */
  public RunShoulder(DoubleSupplier climb) {
    this.climb = climb;
    
    shoulder = new CANSparkMax(11,MotorType.kBrushless);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      double triggerValue = climb.getAsDouble();
      shoulder.set(triggerValue);
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
