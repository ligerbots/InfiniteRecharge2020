/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.Carousel;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
// import jdk.vm.ci.meta.Constant;
import frc.robot.subsystems.DriveTrain;
import frc.robot.commands.ShootFromKey;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.ShooterTuner;
import frc.robot.commands.StopAllShooting;
import frc.robot.commands.TurnAndShoot;
import frc.robot.commands.VisionTargetDistance;
import frc.robot.commands.CarouselCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.EightBallAuto;
import frc.robot.commands.FaceShootingTarget;
import frc.robot.commands.GatherData;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ResetCarousel;
import frc.robot.commands.RunShoulder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
 //import edu.wpi.first.wpilibj.XboxController; will need later
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.GenericHID.Hand;

@SuppressWarnings("all")
/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //private final DriveTrain robotDrive = new DriveTrain();
  private final Throttle throttle = new Throttle();
  private final Turn turn = new Turn();
  private final DriveTrain robotDrive = new DriveTrain();
  public final DriveCommand driveCommand = new DriveCommand(robotDrive, throttle, turn);

  XboxController xbox = new XboxController(0);

  private final Intake intake = new Intake();
  public final Carousel carousel = new Carousel();
  public final Shooter shooter = new Shooter();
  public final Climber climber = new Climber();


  private final Shoulder shoulder = new Shoulder();
  public final RunShoulder runShoulder = new RunShoulder(climber, shoulder);
  public JoystickButton winchRun;

  public CarouselCommand carouselCommand = new CarouselCommand (carousel);

  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  public class Throttle implements DoubleSupplier{

    @Override
    public double getAsDouble() {
      return xbox.getY(Hand.kLeft);
    }
  }

  public class Turn implements DoubleSupplier{

    @Override
    public double getAsDouble() {
      return xbox.getX(Hand.kRight);
    }
  }
  public class Shoulder implements DoubleSupplier{

    @Override
    public double getAsDouble() {
      return xbox.getTriggerAxis(Hand.kRight) - xbox.getTriggerAxis(Hand.kLeft);// set shoulder speed 
    }
  }

  private void configureButtonBindings() {
    JoystickButton xboxA = new JoystickButton(xbox, Constants.XBOX_A);
    JoystickButton xboxB = new JoystickButton(xbox, Constants.XBOX_B);
    JoystickButton xboxX = new JoystickButton(xbox, Constants.XBOX_X);
    JoystickButton xboxY = new JoystickButton(xbox, Constants.XBOX_Y);
    JoystickButton xbox7 = new JoystickButton(xbox, Constants.XBOX_BACK);
    JoystickButton xboxLine = new JoystickButton(xbox, Constants.XBOX_START);
    JoystickButton bumperRight = new JoystickButton(xbox, Constants.XBOX_RB);
    JoystickButton bumperLeft = new JoystickButton(xbox, Constants.XBOX_LB);
    //winchRun = new JoystickButton(xbox, Constants.XBOX_START);
    bumperRight.whileHeld(new IntakeCommand(intake, climber, 0.4));
    bumperLeft.whileHeld(new IntakeCommand(intake, climber, -0.4));
    //xboxB.whileHeld(new ManualCarousel(carousel, carouselCommand));
    xboxA.whenPressed(new ShootFromKey(shooter, carousel, carouselCommand).andThen(new ResetCarousel(carousel, carouselCommand)));
    xboxX.whenPressed(new ShooterCommand(shooter, carousel, robotDrive, 5, carouselCommand, driveCommand).andThen(new ResetCarousel(carousel, carouselCommand)));
    xboxY.whenPressed(new TurnAndShoot(robotDrive, shooter, carousel, carouselCommand, driveCommand));
    //xboxX.whenPressed(new ShooterTuner(shooter));
    //xboxY.whenPressed(new StopAllShooting(shooter));
    JoystickButton xboxStart = new JoystickButton(xbox, Constants.XBOX_START);
    xboxStart.whenPressed(new ShooterTuner(shooter));
    xbox7.whenPressed(new FaceShootingTarget(robotDrive, 3, driveCommand, shooter));
        //xboxA.whenPressed(new ClimberCommand()); //shootercomand
  }


  /*public boolean APressed () {
    return xboxA.get();
  }*/


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  EightBallAuto auto = new EightBallAuto(robotDrive, shooter, carousel, intake, driveCommand);
  public Command getAutonomousCommand() {
     return auto;
  }
}