/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    CANSparkMax intakeMotor;
    public DutyCycleEncoder shoulderPos;
    public AnalogInput rollSpd;
    public DigitalInput limitSwitch;

    public Intake() {
        intakeMotor = new CANSparkMax(5, MotorType.kBrushless);
        shoulderPos = new DutyCycleEncoder(1);
        rollSpd = new AnalogInput(1);
        limitSwitch = new DigitalInput(1);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void run(double speed) {

    }
}