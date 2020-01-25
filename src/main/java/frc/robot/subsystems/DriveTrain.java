/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
    /**
     * Creates a new ExampleSubsystem.
     */
    CANSparkMax leftLeader, leftFollower, rightLeader, rightFollower;
    DifferentialDrive diffDrive;
    boolean fieldCentric = false;
    PIDController turningCtrl;
    double throttleLimit, turnOutput;
    AHRS navX;
    Relay spike;

    enum driveSides {
        LEFT, RIGHT;
    }

    public DriveTrain() {
        leftLeader = new CANSparkMax(14, MotorType.kBrushless);
        leftFollower = new CANSparkMax(14, MotorType.kBrushless);
        rightLeader = new CANSparkMax(14, MotorType.kBrushless);
        rightFollower = new CANSparkMax(14, MotorType.kBrushless);
        diffDrive = new DifferentialDrive(leftLeader, rightLeader);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void allDrive(double x, double y) {
        // diffDrive.arcadeDrive(xSpeed, zRotation);
        // diffDrive.arcadeDrive(throttleLimit, zRotation);
    }

    public void slide(double distance) {

    }
}