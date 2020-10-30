/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Carousel;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
@SuppressWarnings("all")
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  // private DriveTrain driveTrain;
  // private Carousel carousel;
  // private Intake intake;
  // private DriveCommand driveCommand;
  // private Shooter shooter;
  // private Climber climber;
  SendableChooser<Command> chosenAuto = new SendableChooser<>();

  public static int RPMAdjustment;
  public static int HoodAdjustment;
  public static double angleErrorAfterTurn = 0;


  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();

    m_robotContainer.climber.shoulder.setIdleMode(IdleMode.kCoast);
 
    // Set motors to coast so it's easier to move the robot.
    m_robotContainer.robotDrive.setIdleMode(IdleMode.kCoast);

    //m_robotContainer.shooter.calibratePID(0.000085, 0.000000033, 0, 6.776 * 0.00001);

    // Reset Smart Dashboard for shooter test
    SmartDashboard.putString("Shooting", "Idle");

    RPMAdjustment = 0;
    HoodAdjustment = 0;

    
    
    // SmartDashboard.putData(new TestTurret(m_robotContainer.shooter));

    chosenAuto.addDefault("Default Auto", new DriveForwardAuto(m_robotContainer.robotDrive, m_robotContainer.carouselCommand, m_robotContainer.driveCommand));
    chosenAuto.addObject("EightBallAuto", new EightBallAuto(
      m_robotContainer.robotDrive,
      m_robotContainer.shooter,
      m_robotContainer.intake,
      m_robotContainer.climber,
      m_robotContainer.carousel,
      m_robotContainer.driveCommand,
      m_robotContainer.carouselCommand));

    chosenAuto.addObject("ShootAndDrive", new ShootAndDriveAuto(
        m_robotContainer.robotDrive,
        m_robotContainer.shooter,
        m_robotContainer.intake,
        m_robotContainer.climber,
        m_robotContainer.carousel,
        m_robotContainer.driveCommand,
        m_robotContainer.carouselCommand));
  

   SmartDashboard.putData("Chosen Auto", chosenAuto);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the]\[
   * [\]
   * ] mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
    m_robotContainer.climber.shoulder.setIdleMode(IdleMode.kCoast);
    m_robotContainer.climber.winch.setIdleMode(IdleMode.kCoast);
    // Set motors to coast so it's easier to move the robot.
    m_robotContainer.robotDrive.setIdleMode(IdleMode.kCoast);
    m_robotContainer.climber.coastWinch();
  
  }

  @Override
  public void disabledPeriodic() {
    
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_robotContainer.carousel.resetEncoder();
    // Set motors to brake for the drive train
    m_robotContainer.robotDrive.setIdleMode(IdleMode.kBrake);

    m_autonomousCommand = chosenAuto.getSelected();
    //m_robotContainer.carouselCommand.schedule();
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null)
      m_autonomousCommand.schedule();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // Set motors to brake for the drive train
    m_robotContainer.robotDrive.setIdleMode(IdleMode.kBrake);

    SmartDashboard.putNumber("Turret Angle", 75);
    SmartDashboard.putNumber("Target Hood Angle", 60);
    SmartDashboard.putNumber("TSR", -5500);
    System.out.println("teleopInit");

    // Reset the winch encoder
    m_robotContainer.climber.resetWinchEncoder();
    m_robotContainer.climber.winch.setIdleMode(IdleMode.kCoast);


    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    //SmartDashboard.putData(m_robotContainer.testFlup);
    if (m_autonomousCommand != null)
      m_autonomousCommand.cancel();
    m_robotContainer.driveCommand.schedule();
    //m_robotContainer.testFlup.schedule();
    //m_robotContainer.shooter.testSpin();
    m_robotContainer.carouselCommand.schedule();
    //m_robotContainer.testFlup.schedule();
    //m_robotContainer.testIntake.schedule();
    //RunWinch aaa = new RunWinch(m_robotContainer.climber, m_robotContainer);
    //aaa.schedule();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }
}