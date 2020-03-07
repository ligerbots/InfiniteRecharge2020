/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class GatherData extends CommandBase {
  /**
   * Creates a new GatherData.
   */
  File dataFile;
  FileWriter fileWriter;

  public GatherData() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    try {
      dataFile = new File("/home/lvuser/ShooterData.csv");
      if (dataFile.createNewFile()) {
        System.out.println("Created the shooter data file");
      }
      else {
        System.out.println("Found the shooter data file");
      }
      fileWriter = new FileWriter(dataFile, true);
      fileWriter.write(String.format("%5.2f,%5f,%3f%n",
            SmartDashboard.getNumber("Distance", 0),
            SmartDashboard.getNumber("TSR", 0),
            SmartDashboard.getNumber("Target Hood Angle", 0)));
    } catch(IOException e) {
      System.out.println("Error with finding the shooter file");
      e.printStackTrace();
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    try {
      fileWriter.close();
    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
