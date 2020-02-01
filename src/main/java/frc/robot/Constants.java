/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

    //TALON IDs

    // Drive Train SPARK MAXes
    public static final int LEADER_LEFT_CAN_ID = 1; //Drive Train Cansparkmaxes :D

    public static final int LEADER_RIGHT_CAN_ID = 2;

    public static final int FOLLOWER_LEFT_CAN_ID = 3;

    public static final int FOLLOWER_RIGHT_CAN_ID = 4;//Sparkmaxes for drivetrain end here

    // Drive Train Encoders
    public static final int[] LEFT_ENCODER_PORTS = new int[]{1, 2};//DriveTrain Encoder
    
    public static final int[] RIGHT_ENCODER_PORTS = new int[]{3, 4};//DriveTrain Encoder

    //DistanceperPulse

    public static final double DISTANCE_PER_PULSE = 0.0001; // TODO: find real number ~ This is the coefficient of encoder ticks to convert to distance
    
    //SPARKMAX for the spinner (Carousel)
    public static final int SPINNER_CAN_ID = 6; // CAN ID for the spinner for carousel

    public static final double CAROUSEL_SHOOTER_SPEED = 0.5; // This is just percent output

    public static final double CAROUSEL_INTAKE_SPEED = 0.3; // This is just percent output

    // Talon SRX for the grabber (Carousel)
    public static final int GRABBER_TALON_ID = 1; //TALON SRX ID for grabber

    // SPARKMAX for the intake motor (Intake)

    public static final int INTAKE_MOTOR_CAN_ID = 5; //CAN ID for intake motor

    // Intake Color Sensor

    public static final int COLOR_SENSOR_VALUE = 1; // value for color sensor

    // Shooter SPARK MAXES
    public static final int SHOOTER_FLUP_CAN_ID = 35; // Carousel to shooter wheel "FLUP"
    public static final int SHOOTER_ONE_CAN_ID = 32; // Motor 1 on shooter
    public static final int SHOOTER_TWO_CAN_ID = 33; // Motor 2 on shooter
    public static final int SHOOTER_THREE_CAN_ID = 34; // Motor 3 on shooter
    public static final int SHOOTER_SERVO_PWM_ID = 1; // Servo to controll hood angle 

    public static final double WARM_UP_RPM = 3000; // TODO: find a good valuabl
}