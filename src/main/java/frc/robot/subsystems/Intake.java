/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private CANSparkMax intakeMotor = new CANSparkMax(5, MotorType.kBrushless);
    // TODO: Do we need an encoder on the intake or will voltage be enough?
    private ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
    // TODO: There was some discussion of a beam break to help monitor balls
    private DigitalInput beamBreak = new DigitalInput(1), limitSwitch = new DigitalInput(2);

    public Intake() {

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void run(double speed) {
        intakeMotor.set(speed);
    }

    public Color getColor() {
        return colorSensor.getColor();
    }
}