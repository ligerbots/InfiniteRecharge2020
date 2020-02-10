/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
@SuppressWarnings("all")
public class Shooter extends SubsystemBase {

    CANSparkMax motor1, motor2, motor3, flup;

    CANEncoder shooterEncoder;

    Servo hoodServo, turretServo;

    private HashMap<Double,Double> distanceLookUp = new HashMap<Double,Double>() {}; //set up lookup table for ranges

    CANPIDController pidController;
    // TODO: Need to add velocity PID for shooter



    public Shooter() {

        motor1 = new CANSparkMax(Constants.SHOOTER_ONE_CAN_ID,MotorType.kBrushless);
        motor2 = new CANSparkMax(Constants.SHOOTER_TWO_CAN_ID,MotorType.kBrushless);
        motor3 = new CANSparkMax(Constants.SHOOTER_THREE_CAN_ID,MotorType.kBrushless);

        flup = new CANSparkMax(Constants.SHOOTER_FLUP_CAN_ID,MotorType.kBrushless);

        hoodServo = new Servo(Constants.SHOOTER_SERVO_PWM_ID);

        turretServo = new Servo(Constants.SHOOTER_TURRET_SERVO_ID);

        shooterEncoder = new CANEncoder(motor1);

        pidController = new CANPIDController(motor1);

        pidController.setFeedbackDevice(shooterEncoder);

        motor2.follow(motor1);  //  We want motor1 to be master and motor2 and 3 follow the speed of motor1
        motor3.follow(motor1);

        //TODO: Populate lookup table
        
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run every 20ms
    }

    public void prepareShooter(final double distance) {

        pidController.setReference(calculateShooterSpeed(distance), ControlType.kVelocity);
        hoodServo.setAngle(calculateShooterHood(distance));
        // TODO: The idea was that this would set the shooter speed
        // and hoodServo value based on the input distance.
    }

    public void shoot () {
        flup.set(0.5); // TODO: figure out what to do with this constant
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
        return hoodServo.getAngle() > targetAngle - 1 && hoodServo.getAngle() < targetAngle + 1;
    }

    public void calibratePID (final double p, final double i, final double d) {
        pidController.setP(p);
        pidController.setI(i);
        pidController.setD(d);
    }

    public void stopAll () {
        pidController.setReference(0, ControlType.kVoltage);
        hoodServo.setAngle(0);
    }

    public double getTurretAngle () {
        return turretServo.get() *  Constants.TURRET_ANGLE_COEFFICIENT;
    }

    public void setTurret (double angle) {
        turretServo.set(angle / Constants.TURRET_ANGLE_COEFFICIENT);
    }
}