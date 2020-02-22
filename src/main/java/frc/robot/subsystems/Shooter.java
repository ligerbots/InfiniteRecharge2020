/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;

import java.util.Arrays;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
@SuppressWarnings("all")
public class Shooter extends SubsystemBase {

    CANSparkMax motor1, motor2, motor3;

    static CANSparkMax flup;

    CANEncoder shooterEncoder;

    Servo hoodServo, turretServo;

    private HashMap<Double,Double> distanceLookUp = new HashMap<Double,Double>() {}; //set up lookup table for ranges

    CANPIDController pidController;

    Relay spike;


    public Shooter() {

        motor1 = new CANSparkMax(Constants.SHOOTER_ONE_CAN_ID,MotorType.kBrushless);
        motor2 = new CANSparkMax(Constants.SHOOTER_TWO_CAN_ID,MotorType.kBrushless);
        motor3 = new CANSparkMax(Constants.SHOOTER_THREE_CAN_ID,MotorType.kBrushless);

        flup = new CANSparkMax(Constants.SHOOTER_FLUP_CAN_ID, MotorType.kBrushless);

        // Set motors to brake when idle. We don't want the drive train to coast.
        Arrays.asList(motor1, motor2, motor3, flup)
                .forEach((CANSparkMax spark) -> spark.setIdleMode(IdleMode.kCoast));


        hoodServo = new Servo(Constants.SHOOTER_SERVO_PWM_ID);

        turretServo = new Servo(Constants.SHOOTER_TURRET_SERVO_ID);

        shooterEncoder = new CANEncoder(motor2);
        shooterEncoder.setVelocityConversionFactor(2.666);

        pidController = new CANPIDController(motor2);

        pidController.setFeedbackDevice(shooterEncoder);


        motor1.follow(motor2, true);  //  We want motor1 to be master and motor2 and 3 follow the speed of motor1
        motor3.follow(motor2);

        spike = new Relay(0);

        //TODO: Populate lookup table
        
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter RPM", getSpeed());
    }

    public double getVoltage() {
        return motor2.getBusVoltage();
    }

    public void setHood (double angle) {
        System.out.println("hood angle SET!!!!");
        if (angle < 40) {
            angle = 40;
        }
        if (angle > 160) {
            angle = 160;
        }
        hoodServo.setAngle(angle);
    }

    public double getSpeed () {
        return shooterEncoder.getVelocity();
    }

    public void prepareShooter(final double distance) {

        pidController.setReference(calculateShooterSpeed(distance), ControlType.kVelocity);
        hoodServo.setAngle(calculateShooterHood(distance));
        // TODO: The idea was that this would set the shooter speed
        // and hoodServo value based on the input distance.
    }

    public void shoot () {
        flup.set(-0.5); // TODO: figure out what to do with this constant
    }

    public void testSpin () {
        pidController.setReference(-4000, ControlType.kVelocity);
        SmartDashboard.putString("Shooting", "Shooting");
    }

    public void setShooterRPM (double rpm) {
        System.out.println("Shooter RPM SET!!!!!");
        pidController.setReference(rpm, ControlType.kVelocity);
    }

    public double calculateShooterSpeed (final double distance) {
        // TODO: Some logic 
        return 0;
    }

    public double calculateShooterHood (final double distance) {
        // TODO: Some logic (should return degrees)
        final float maxShootingDistance = 40; // set placeholder for max shooting distance (in feet)(YJ)
        return 0.0; // (YJ) TODO: determine how to integrate angle slope equation into calculations
        
    }

    public void warmUp () {
        pidController.setReference(Constants.WARM_UP_RPM, ControlType.kVelocity);
    }

    public boolean speedOnTarget (final double targetVelocity, final double percentAllowedError) {
        final double max = targetVelocity * (1.0 + (percentAllowedError / 100.0));
        final double min = targetVelocity * (1.0 - (percentAllowedError / 100.0));
        return shooterEncoder.getVelocity() > min && shooterEncoder.getVelocity() <  max; 
    }

    public boolean hoodOnTarget (final double targetAngle) {
        System.out.println("ServoPosition: " + hoodServo.getPosition());
        return hoodServo.getAngle() > targetAngle - 0.5 && hoodServo.getAngle() < targetAngle + 0.5;
    }

    public void calibratePID (final double p, final double i, final double d) {
        pidController.setP(p);
        pidController.setI(i);
        pidController.setD(d);
    }

    public void stopAll () {
        pidController.setReference(0, ControlType.kVelocity);
        flup.set(0);
        hoodServo.setAngle(160);
    }

    public double getTurretAngle () {
        return turretServo.get() *  Constants.TURRET_ANGLE_COEFFICIENT;
    }

    public void setTurret (double angle) {
        turretServo.setAngle(angle);
    }

    public void setLEDRing (boolean on) {
        spike.set(on ? Value.kForward : Value.kReverse);
    }
}