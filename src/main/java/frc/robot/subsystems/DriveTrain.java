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

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
@SuppressWarnings("all")
public class DriveTrain extends SubsystemBase {

    private CANSparkMax leftLeader = new CANSparkMax(Constants.LEADER_LEFT_CAN_ID, MotorType.kBrushless);
    private CANSparkMax leftFollower = new CANSparkMax(Constants.FOLLOWER_LEFT_CAN_ID, MotorType.kBrushless);
    private CANSparkMax rightLeader = new CANSparkMax(Constants.LEADER_RIGHT_CAN_ID, MotorType.kBrushless);
    private CANSparkMax rightFollower = new CANSparkMax(Constants.FOLLOWER_RIGHT_CAN_ID, MotorType.kBrushless);

    private final SpeedControllerGroup leftMotors = new SpeedControllerGroup(leftLeader, leftFollower);

    private final SpeedControllerGroup rightMotors = new SpeedControllerGroup(rightLeader, rightFollower);

    DifferentialDrive robotDrive;
    DifferentialDriveOdometry odometry;

    Encoder leftEncoder = new Encoder(Constants.LEFT_ENCODER_PORTS[0], Constants.LEFT_ENCODER_PORTS[1]);
    Encoder rightEncoder = new Encoder(Constants.RIGHT_ENCODER_PORTS[0], Constants.RIGHT_ENCODER_PORTS[1]);

    AHRS navX;

    double limitedThrottle;

    public DriveTrain() {

        // TODO: Verify which motors need to be inverted
        // Since we're using DifferentialDrive below, we should not need to invert any,
        // but it doesn't hurt to be explicit.
        leftLeader.setInverted(false);
        rightLeader.setInverted(false);

        robotDrive = new DifferentialDrive(leftMotors, rightMotors);
        robotDrive.setSafetyEnabled(false);

        navX = new AHRS(Port.kMXP, (byte) 200);

        // Set current limiting on drve train to prevent brown outs
        Arrays.asList(leftLeader, leftFollower, rightLeader, rightFollower)
                .forEach((CANSparkMax spark) -> spark.setSmartCurrentLimit(40));

        // Set motors to brake when idle. We don't want the drive train to coast.
        Arrays.asList(leftLeader, leftFollower, rightLeader, rightFollower)
                .forEach((CANSparkMax spark) -> spark.setIdleMode(IdleMode.kBrake));

        //TODO determine real numbers to use here
        rightLeader.setOpenLoopRampRate(0.0065);
        leftLeader.setOpenLoopRampRate(0.0065);

        ////////////////////////////ODOMETRY SET UP//////////////////////////////////

        leftEncoder.setDistancePerPulse(Constants.DISTANCE_PER_PULSE);
        rightEncoder.setDistancePerPulse(Constants.DISTANCE_PER_PULSE);

        odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(0));
    }

    public Pose2d getPose () {
        return odometry.getPoseMeters();
    }

    public void tankDriveVolts (double leftVolts, double rightVolts) {
        leftMotors.setVoltage(leftVolts);
        rightMotors.setVoltage(-rightVolts);// make sure right is negative becuase sides are opposite
        robotDrive.feed();
      }
    
    public double getAverageEncoderDistance() {
        return (leftEncoder.getDistance() + rightEncoder.getDistance()) / 2.0;
    }
    
    public double getLeftEncoderDistance() {
        return leftEncoder.getDistance();
    }
    
    public double getRightEncoderDistance() {
        return rightEncoder.getDistance();
    }
    
    public double getHeading() {
        return Math.IEEEremainder(navX.getAngle(), 360) * -1; // -1 here for unknown reason look in documatation
    }

    public void resetHeading() {
        navX.reset();
    }
    
    public void resetEncoders () {
        leftEncoder.reset();
        rightEncoder.reset();
    }
    
    public void resetOdometry (Pose2d pose) {
        resetEncoders();
        resetHeading();
        odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds () {
        return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void allDrive(double throttle, double rotate) {
        // TODO: This will be based on the intake position. We don't have an elevator this year.
        /*if (Robot.elevator.getPosition() > 40)
            limitedThrottle = Math.abs(throttle) > 0.5 ? 0.5 * Math.signum(throttle) : throttle;
        else
            limitedThrottle = throttle;
        robotDrive.arcadeDrive(limitedThrottle, -rotate);*/
    }

    public void slide(double distance) {

    }
}