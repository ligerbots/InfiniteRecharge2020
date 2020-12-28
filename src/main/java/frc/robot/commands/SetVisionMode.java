/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Vision;

public class SetVisionMode extends InstantCommand {
    /**
     * Sets the Vision processing into a specific mode.
     */
    
    private Vision m_visionSystem;
    private Vision.VisionMode m_desiredMode;

    public SetVisionMode(Vision visionSystem, Vision.VisionMode mode) {
        m_visionSystem = visionSystem;
        m_desiredMode = mode;

        addRequirements(visionSystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_visionSystem.setMode(m_desiredMode);
    }
}
