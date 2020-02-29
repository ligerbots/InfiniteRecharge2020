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
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.PIDBase.Tolerance;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    public PIDController turnSpeedController;

    public double turnOutput;

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

        robotDrive = new DifferentialDrive(leftMotors, rightMotors);
        robotDrive.setSafetyEnabled(false);

        navX = new AHRS(Port.kMXP, (byte) 200);

        // Set current limiting on drve train to prevent brown outs
        Arrays.asList(leftLeader, leftFollower, rightLeader, rightFollower)
                .forEach((CANSparkMax spark) -> spark.setSmartCurrentLimit(35));

        // Set motors to brake when idle. We don't want the drive train to coast.
        Arrays.asList(leftLeader, leftFollower, rightLeader, rightFollower)
                .forEach((CANSparkMax spark) -> spark.setIdleMode(IdleMode.kBrake));

        //TODO determine real numbers to use here
        //rightLeader.setOpenLoopRampRate(0.0065);
        //leftLeader.setOpenLoopRampRate(0.0065);

        ////////////////////////////ODOMETRY SET UP//////////////////////////////////

        leftEncoder.setDistancePerPulse(Constants.DISTANCE_PER_PULSE);
        rightEncoder.setDistancePerPulse(Constants.DISTANCE_PER_PULSE);

        odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(0));

        turnSpeedController = new PIDController(0.015, 0.0001, 0.0, 0, navX, output -> this.turnOutput = output);

        SmartDashboard.putString("vision/active_mode/selected", "goalfinder");

    }

    public Pose2d getPose () {
        return odometry.getPoseMeters();
    }

    public void tankDriveVolts (double leftVolts, double rightVolts) {
        leftMotors.setVoltage(-leftVolts);
        rightMotors.setVoltage(rightVolts);// make sure right is negative becuase sides are opposite
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

    public void enableTurningControl(double angle, double tolerance) {
        double angleOffset = angle;
        double startAngle = getHeading();
        double targetAngle = startAngle + angle;
    
        // We need to keep all angles between -180 and 180. Account for that here
        // wrapCorrection will be used below in turnError to undo what we do here
        double originalTargetAngle = targetAngle;
        if (targetAngle > 180.0) {
            targetAngle -= 360.0;
        }
        else if (targetAngle < -180.0) {
            targetAngle += 360.0;
        }
        
        turnSpeedController.setSetpoint(targetAngle);
        turnSpeedController.enable();
        turnSpeedController.setInputRange(-180.0, 180.0);
        turnSpeedController.setAbsoluteTolerance(tolerance);
        turnSpeedController.setOutputRange(-1.0, 1.0);
        turnSpeedController.setContinuous(true);


        System.out.printf(
            "currentAngle: %5.2f, originalTargetAngle: %5.2f, targetAngle: %5.2f, ",
            startAngle, originalTargetAngle, targetAngle);
  }

    public DifferentialDriveWheelSpeeds getWheelSpeeds () {
        return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
    }

    @Override
    public void periodic() {
        odometry.update(Rotation2d.fromDegrees(getHeading()), leftEncoder.getDistance(), rightEncoder.getDistance());
        SmartDashboard.putNumber("Heading", getHeading());
        SmartDashboard.putString("Pose", getPose().toString());
        // SmartDashboard.putNumber("Vision Angle", SmartDashboard.getNumberArray("vision/target_info", new Double[]{0.0,0.0})[4] * 180.0 / 3.1416);
        //SmartDashboard.putNumber("Arc tan adjustment", Math.atan(7.5 / SmartDashboard.getNumberArray("vision/target_info", new Double[]{0.0,0.0})[3]));

        // This method will be called once per scheduler run
    }

    public void allDrive(double throttle, double rotate, boolean squaredInputs) {
        if (squaredInputs) {
            if (Math.abs(throttle) < 0.1)
                throttle = 0;
            if (Math.abs(rotate) < 0.1) 
                rotate = 0;
        }
        robotDrive.arcadeDrive(throttle, -rotate, squaredInputs);
    }

    public int getLeftEncoderTicks () {
        return leftEncoder.get();
    }

    public int getRightEncoderTicks () {
        return rightEncoder.get();
    }

    public double turnSpeedCalc(double angleError) {
        if (Math.abs(angleError) > 60) {
            return 0.8 * Math.signum(angleError);
        }
        else if (Math.abs(angleError) > 30) {
            return 0.4 * Math.signum(angleError);
        }
        else if (Math.abs(angleError) > 10) {
            return 0.1 * Math.signum(angleError);
        }
        else if (Math.abs(angleError) > 5) {
            return 0.065 * Math.signum(angleError);
        }
        else {
            return 0.065 * Math.signum(angleError);
        }
    }

    public double getPitch() {
        return navX.getPitch();
    }

    public void setIdleMode(IdleMode idleMode) {
            Arrays.asList(leftLeader, leftFollower, rightLeader, rightFollower)
            .forEach((CANSparkMax spark) -> spark.setIdleMode(idleMode));


    }
}