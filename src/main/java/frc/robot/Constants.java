/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

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
    public static final int LEADER_RIGHT_CAN_ID = 3;
    public static final int FOLLOWER_LEFT_CAN_ID = 2;
    public static final int FOLLOWER_RIGHT_CAN_ID = 4;//Sparkmaxes for drivetrain end here

    // Drive Train Encoders
    public static final int[] LEFT_ENCODER_PORTS = new int[]{0, 1};//DriveTrain Encoder
    public static final int[] RIGHT_ENCODER_PORTS = new int[]{2, 3};//DriveTrain Encoder

    //DistanceperPulse
    
    //Talon SRX for the Carousel
    public static final int CAROUSEL_CAN_ID = 12; // CAN ID for the spinner for carousel
    public static final double CAROUSEL_SHOOTER_SPEED = 0.8; // This is just percent output
    public static final double CAROUSEL_INTAKE_SPEED = 0.7; // This is just percent output

    // Talon SRX for the grabber (Carousel)
    public static final int GRABBER_TALON_ID = 1; //TALON SRX ID for grabber

    // SPARKMAX for the intake motor (Intake)
    public static final int INTAKE_MOTOR_CAN_ID = 10; //CAN ID for intake motor
    public static final double INTAKE_SPEED = 0.4;

    // Intake Color Sensor

    public static final int COLOR_SENSOR_VALUE = 1; // value for color sensor

    // Shooter SPARK MAXES
    public static final int SHOOTER_FLUP_CAN_ID = 8; // Carousel to shooter wheel "FLUP"
    public static final int SHOOTER_ONE_CAN_ID = 5; // Motor 1 on shooter
    public static final int SHOOTER_TWO_CAN_ID = 6; // Motor 2 on shooter
    public static final int SHOOTER_THREE_CAN_ID = 7; // Motor 3 on shooter
    public static final int SHOOTER_SERVO_PWM_ID = 0; // Servo to controll hood angle 
    public static final int SHOOTER_TURRET_SERVO_ID = 1; // TODO: Fix this

    public static final double FLUP_STOP_CURRENT = 12; //TODO: FIX THIS VALUE

    public static final double TURRET_ANGLE_COEFFICIENT = 180; // TODO: Fix this

    public static final double WARM_UP_RPM = 3000; // TODO: find a good valuabl

    public static final double MAX_TURRET_OFFSET = 10; // in degrees (may be more)
    
    // Set Shoulder parameters
    public static final int SHOULDER_MOTOR_CAN_ID = 11;; // shoulder for the climber
    public static final double SHOULDER_MIN_HEIGHT = 0.44;
    // If arm gets below 10 degrees above min height, we'll either stop the motor and let if fall in brake mode
    // or we'll just move t down really slowly.
    public static final double SHOULDER_MIN_VELOCITY_BUFFER = 2.0/360.0; // 2 degrees;
    public static final double SHOULDER_MIN_VELOCITY_HEIGHT = SHOULDER_MIN_HEIGHT + SHOULDER_MIN_VELOCITY_BUFFER;
    public static final double SHOULDER_MAX_HEIGHT = 0.604; 
    public static final double SHOULDER_CLIMB_HEIGHT = 0.62;
    public static final double SHOULDER_RELEASE_HEIGHT = 0.6;
    public static final double SHOULDER_START_HEIGHT = 0.59;
    public static final double SHOULDER_HEIGHT_FOR_SPRING_TO_LIFT = 0.25;
    public static final double SHOULDER_HEIGHT_FOR_RAISE1 = 0.48;
    public static final double SHOULDER_HEIGHT_FOR_FRAME_PERIMETER = 0.56 - 2.0/360.0; 
    public static final double SHOULDER_HEIGHT_FOR_MAX_CLIMB = 0.62;

    public static final double SHOULDER_SPEED_UP = 4.0; //VOLTAGE~!!!!!
    public static final double SHOULDER_SPEED_HOLD = 0.3
    ;
    public static final double SHOULDER_RATE_DOWN = 40.0/360.0 * 12;
    // NOTE: This has to be negative to keep the robot level during climb
    public static final double SHOULDER_SPEED_LEVEL = -5.5;

    public static final double ROBOT_PITCH_ANGLE_FOR_CLIMB = 5.0;

    public static final int CAROUSEL_FIFTH_ROTATION_TICKS = 12561;
    public static final int WINCH_MOTOR_CAN_ID = 9; // Winch for the climber
    public static final double WINCH_SPEED_FAST = 10.0; // winch speed for going fast
    public static final double WINCH_SPEED_SLOW = 0.5; // winch speed for going slow
    public static final double WINCH_SPEED_CLIMB = 0.8; //winch speed for actually climbing

    public static final double WINCH_MAX_HEIGHT_TICK_COUNT = 665;
    public static final double WINCH_HEIGHT_FOR_LEVEL_BAR_AT_FRAME_PERIMETER = 200; // TODO: Measure this
    public static final double WINCH_LEVEL_BAR_TICK_COUNT = 862;
    public static final double WINCH_CLIMB_HEIGHT = 1262;

    // xbox button mapimng 
    public static final int XBOX_A = 1;
    public static final int XBOX_B = 2;
    public static final int XBOX_X = 3;
    public static final int XBOX_Y = 4;

    // bumpers
    public static final int XBOX_LB = 5;
    public static final int XBOX_RB = 6;
    
    public static final int XBOX_BACK = 7;
    public static final int XBOX_START = 8;

    // joy stick button
    public static final int XBOX_JL = 9;
    public static final int XBOX_JR = 10;

    // AUTO CHARACTERIZATION CONSTANTS

    public static final double DISTANCE_PER_PULSE = 0.00155852448;

    //FEEDFORWARD AND FEEDBACK GAINS
    public static final double ksVolts = 0.182; 
    public static final double kvVoltSecondsPerMeter = 2.64; 
    public static final double kaVoltSecondsSquaredPerMeter = 0.324; 
    public static final double kPDriveVel = 6; 

    //DIFFERENTIAL DRIVE KINEMATICS
    public static final double kTrackwidth = 0.6604; // in meters
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidth);

    //MAX TRAJECTORY VELOCITY AND ACCELERATION
    public static final double kMaxSpeed = 1.75; // TODO: ASSIGN A REAL VALUE meters per second
    public static final double kMaxAcceleration = 1.5; // TODO: ASSIGN A REAL VALUE meters per second per second

    //RAMSETE PARAMETERS
    public static final double kRamseteB = 2; // generic ramsete values
    public static final double kRamseteZeta = 0.7; // generic ramsete values

}