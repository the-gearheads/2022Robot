// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.AutonDrive;
import frc.robot.commands.Shoot;
import frc.robot.commands.TurnToAngle;
import frc.robot.commands.SetDistance;
import frc.robot.subsystems.DistanceSensor;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  // private final DriveTrain driveTrain = new DriveTrain();
  // private final Shooter shooter = new Shooter();
  private final XboxController controller = new XboxController(Constants.Controller.PORT);
  private final XboxController joystick = new XboxController(Constants.Joystick.PORT);
  private final DriveTrain driveTrain = new DriveTrain();
  private final DistanceSensor distanceSensor = new DistanceSensor();

  // private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem)
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    Timer timer = new Timer();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // JoystickButton btn1 = new JoystickButton(controller, 1);
    // btn1.whenPressed(new InstantCommand(driveTrain::zeroEncoders, driveTrain));
    // JoystickButton btn1 = new JoystickButton(controller, 1);
    // btn1.whenPressed(()->{(new Shoot(shooter)).schedule(false);});
    JoystickButton btn2 = new JoystickButton(controller, 2);
    btn2.whenPressed(new InstantCommand(()->
    {SmartDashboard.putNumber("extension time", 1);}));
    JoystickButton btn4 = new JoystickButton(controller, 4);
    btn4.whenPressed(new SetDistance(driveTrain, distanceSensor, 45));
    JoystickButton btn1 = new JoystickButton(controller, 1);
    btn1.whenPressed(new ArcadeDrive(driveTrain));
    JoystickButton btn3 = new JoystickButton(controller, 3);
    btn3.whenPressed(new InstantCommand(()->{
      double requestedDistance = distanceSensor.getDistanceInCM() - 150;
      driveTrain.zeroEncoders();
      (new AutonDrive(driveTrain, new ArrayList<Translation2d>(), new Pose2d(requestedDistance / 100.0, 0, new Rotation2d(0)))).schedule(false);
    },driveTrain, distanceSensor));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    // return m_autoCommand;
    SmartDashboard.putBoolean("Started", true);
    InstantCommand zeroEncoders = new InstantCommand(()->{
      driveTrain.zeroEncoders();
      Timer timer = new Timer();
      timer.start();
      while(timer.get() < 3){

      }
      timer.stop();
    }, driveTrain);
    AutonDrive firstMovement = new AutonDrive(driveTrain, new ArrayList<Translation2d>(), new Pose2d(-3,0,new Rotation2d(0)));
    TurnToAngle turnToAngle = new TurnToAngle(driveTrain,180);
    AutonDrive secondMovement = new AutonDrive(driveTrain, new ArrayList<Translation2d>(), new Pose2d(-0.3,0,new Rotation2d(Math.PI)));
    TurnToAngle turnToAngle2 = new TurnToAngle(driveTrain,180);


    
    return (new SequentialCommandGroup(zeroEncoders, firstMovement, turnToAngle, secondMovement, turnToAngle2));
    // return null;
  }
}
