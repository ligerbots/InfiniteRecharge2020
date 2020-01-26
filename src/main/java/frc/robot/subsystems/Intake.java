/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANDigitalInput.LimitSwitch;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    CANSparkMax intakeMotor;
    public AnalogInput rollSpd;
    public ColorSensorV3 colorSensor;
    public LimitSwitch limitSwitch;

    public Intake() {
        intakeMotor = new CANSparkMax(5, MotorType.kBrushless);
        rollSpd = new AnalogInput(1);
        colorSensor = new ColorSensorV3(null);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void run(double speed) {

    }
}