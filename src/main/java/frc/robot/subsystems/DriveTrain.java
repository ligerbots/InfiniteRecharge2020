/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;

import java.util.Arrays;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class DriveTrain extends SubsystemBase {
    CANSparkMax leftLeader, leftFollower, rightLeader, rightFollower;
    DifferentialDrive diffDrive;
    double limitedThrottle;
    AHRS navX;

    public DriveTrain() {
        leftLeader = new CANSparkMax(1, MotorType.kBrushless);
        leftFollower = new CANSparkMax(2, MotorType.kBrushless);
        rightLeader = new CANSparkMax(3, MotorType.kBrushless);
        rightFollower = new CANSparkMax(4, MotorType.kBrushless);
        leftLeader.setInverted(false);
        leftFollower.follow(leftLeader, false);
        rightFollower.follow(rightLeader);
        diffDrive = new DifferentialDrive(leftLeader, rightLeader);
        diffDrive.setSafetyEnabled(false);
        navX = new AHRS(Port.kMXP, (byte) 200);
        Arrays.asList(leftLeader, leftFollower, rightLeader, rightFollower)
                .forEach((CANSparkMax spark) -> spark.setSmartCurrentLimit(40));
        Arrays.asList(leftLeader, leftFollower, rightLeader, rightFollower)
                .forEach((CANSparkMax spark) -> spark.setIdleMode(IdleMode.kBrake));
        rightLeader.setOpenLoopRampRate(0.0065);
        leftLeader.setOpenLoopRampRate(0.0065);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void allDrive(double throttle, double rotate) {
        if (Robot.elevator.getPosition() > 40)
            limitedThrottle = Math.abs(throttle) > 0.5 ? 0.5 * Math.signum(throttle) : throttle;
        else
            limitedThrottle = throttle;
        diffDrive.arcadeDrive(limitedThrottle, -rotate);
    }

    public void slide(double distance) {

    }
}