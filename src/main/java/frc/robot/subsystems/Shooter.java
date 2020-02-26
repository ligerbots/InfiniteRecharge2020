/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot.subsystems;

import java.util.Arrays;
import java.io.BufferedReader;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.TreeMap;
import java.util.Map.Entry;

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
public class Shooter extends SubsystemBase {

    CANSparkMax motor1, motor2, motor3;
    static CANSparkMax flup;
    CANEncoder shooterEncoder;
    Servo hoodServo, turretServo;
    private TreeMap<Double, Double[]> distanceLookUp = new TreeMap<Double,Double[]>() {}; //set up lookup table for ranges
    private TreeMap<Double, Double> turretAngleLookup = new TreeMap<Double, Double>() {};
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

        motor1.setSmartCurrentLimit(40);
        motor2.setSmartCurrentLimit(40);
        motor3.setSmartCurrentLimit(40);

        try (BufferedReader br = new BufferedReader(new FileReader("/home/lvuser/ShooterData.csv"))) {
            String line;
            while ((line = br.readLine()) != null) {
                String values_str[] = line.split(",");
                double values[] = new double[values_str.length];
                for(int i = 0; i < values.length; i++) {
                    values[i] = Double.parseDouble(values_str[i].trim());
                    distanceLookUp.put(values[0], new Double[] {values[1], values[2]});
                  }               
            }
        }
        catch (Exception e) {
            System.err.println("Error trying to read or parse ShooterData.csv: " + e.getMessage()); 
            System.err.println("Using original hard-coded table instead");

            distanceLookUp.put(new Double(112.6), new Double[] {new Double(-5500), new Double(90)});
            distanceLookUp.put(new Double(137.1), new Double[] {new Double(-5500), new Double(80)});
            distanceLookUp.put(new Double(168.9), new Double[] {new Double(-6000), new Double(70)});
            distanceLookUp.put(new Double(227.0), new Double[] {new Double(-7000), new Double(65)});
            distanceLookUp.put(new Double(318.1), new Double[] {new Double(-8000), new Double(60)});
            distanceLookUp.put(new Double(253.4), new Double[] {new Double(-7500), new Double(60)});
            distanceLookUp.put(new Double(235.2), new Double[] {new Double(-7500), new Double(55)});            
        }
      
        turretAngleLookup.put(0.0, 72.0);
        turretAngleLookup.put(1.0, 77.0);
        turretAngleLookup.put(2.0, 82.0);
        turretAngleLookup.put(3.0, 85.0);
        turretAngleLookup.put(-4.0, 59.0);
        turretAngleLookup.put(-2.0, 66.0);
        turretAngleLookup.put(-3.0, 62.5);
        turretAngleLookup.put(4.0, 89.0);
        turretAngleLookup.put(5.0, 94.0);
        turretAngleLookup.put(-5.0, 53.0);





    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter RPM", getSpeed());
        SmartDashboard.putNumber("Shooter motor current", motor2.getOutputCurrent());
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

        pidController.setReference(-calculateShooterSpeed(distance), ControlType.kVelocity);
        hoodServo.setAngle(calculateShooterHood(distance));
        // TODO: The idea was that this would set the shooter speed
        // and hoodServo value based on the input distance.
    }

    public void setShooterVoltage (double voltage) {
        pidController.setReference(voltage, ControlType.kVoltage);
    }

    public void shoot () {
        System.out.println("Flup current: " + flup.getOutputCurrent());
        if (flup.getOutputCurrent() < Constants.FLUP_STOP_CURRENT) {
            flup.set(-0.5);
        }
        else {
            flup.set(0);
        }
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
        Entry<Double, Double[]> floorEntry = distanceLookUp.floorEntry(distance);
        Entry<Double, Double[]> ceilingEntry = distanceLookUp.higherEntry(distance);
        System.out.format("Distance Floor %4.1f%n", floorEntry.getKey());
        System.out.format("Distance current %4.1f%n", distance);
        System.out.format("Distance current %4.1f", ceilingEntry.getKey());

        if (floorEntry != null && ceilingEntry != null) {
            // Charles' calculation
            double ratio = 1 - (ceilingEntry.getKey() - distance) / (ceilingEntry.getKey() - floorEntry.getKey());
            System.out.format("Ratio %4.1f", ratio);
            double result = floorEntry.getValue()[0] + ratio * (ceilingEntry.getValue()[0] - floorEntry.getValue()[0]);
            System.out.format("Interpolated shooter speed %4.1f", result);

            // Mark's calculation
            // double result = (ceilingEntry.getValue()[0] - floorEntry.getValue()[0]) / 
            //                     (ceilingEntry.getKey() - floorEntry.getKey())
            //                   * (ceilingEntry.getKey() - distance) / 
            //                     (ceilingEntry.getKey() - floorEntry.getKey()) + floorEntry.getValue()[0];
            return result;
        }
        else {
            return -1000;
        }
    }

    public double calculateShooterHood (final double distance) {
        Entry<Double, Double[]> floorEntry = distanceLookUp.floorEntry(distance);
        Entry<Double, Double[]> ceilingEntry = distanceLookUp.higherEntry(distance);

        if (floorEntry != null && ceilingEntry != null) {
            // Charles calculation
            double ratio = 1 - (ceilingEntry.getKey() - distance) / (ceilingEntry.getKey() - floorEntry.getKey());
            double result = floorEntry.getValue()[1] + ratio * (ceilingEntry.getValue()[1] - floorEntry.getValue()[1]);

            // Mark's calculation
            // double result = (ceilingEntry.getValue()[1] - floorEntry.getValue()[1]) / (ceilingEntry.getKey() - floorEntry.getKey()) * (ceilingEntry.getKey() - distance) / (ceilingEntry.getKey() - floorEntry.getKey())  + floorEntry.getValue()[1];
            System.out.format("Interpolated Hood Angle %4.1f%n", result);

            return result;
        }
        else {
            return 60;
        }
    }

    public void warmUp () {
        pidController.setReference(Constants.WARM_UP_RPM, ControlType.kVelocity);
    }

    public boolean speedOnTarget (final double targetVelocity, final double percentAllowedError) {
        final double max = targetVelocity * (1.0 + (percentAllowedError / 100.0));
        final double min = targetVelocity * (1.0 - (percentAllowedError / 100.0));
        return shooterEncoder.getVelocity() > max && shooterEncoder.getVelocity() < min;  //this is wack cause it's negative
    }

    public boolean hoodOnTarget (final double targetAngle) {
        System.out.println("ServoPosition: " + hoodServo.getPosition());
        return hoodServo.getAngle() > targetAngle - 0.5 && hoodServo.getAngle() < targetAngle + 0.5;
    }

    public void calibratePID (final double p, final double i, final double d, final double f) {
        pidController.setP(p);
        pidController.setI(i);
        pidController.setD(d);
        pidController.setFF(f);
    }

    public void stopAll () {
        pidController.setReference(0, ControlType.kVoltage);
        flup.set(0);
        hoodServo.setAngle(160);
    }

    public double getTurretAngle () {
        return turretServo.get() *  Constants.TURRET_ANGLE_COEFFICIENT;
    }

    public void setTurret (double angle) {
        System.out.println("Moving turret to " + angle);
        turretServo.setAngle(angle);
    }

    public void setTurretAdjusted(double adjustedAngle) {
        Entry<Double, Double> floorEntry = turretAngleLookup.floorEntry(adjustedAngle);
        Entry<Double, Double> ceilingEntry = turretAngleLookup.higherEntry(adjustedAngle);
        if (floorEntry != null && ceilingEntry != null) {
            // Charles calculation
            double ratio = 1 - (ceilingEntry.getKey() - adjustedAngle) / (ceilingEntry.getKey() - floorEntry.getKey());
            double result = floorEntry.getValue() + ratio * (ceilingEntry.getValue() - floorEntry.getValue());

            // Mark's calculation
            // double result = (ceilingEntry.getValue()[1] - floorEntry.getValue()[1]) / (ceilingEntry.getKey() - floorEntry.getKey()) * (ceilingEntry.getKey() - distance) / (ceilingEntry.getKey() - floorEntry.getKey())  + floorEntry.getValue()[1];
            System.out.format("Interpolated Hood Angle %4.1f%n", result);

            turretServo.setAngle(result);
        }
        else {
            turretServo.setAngle(72);
        }
    }

    public void setLEDRing (boolean on) {
        spike.set(on ? Value.kForward : Value.kReverse);
    }
}