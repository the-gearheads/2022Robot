// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

import javax.swing.text.DefaultStyledDocument.ElementSpec;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.Auton.AutonDrive;
import frc.robot.commands.Auton.WriteAuton;
import frc.robot.commands.Auton.PreparedAuton;
import frc.robot.commands.Auton.ReadAuton;
import frc.robot.commands.Auton.Wait;
import frc.robot.commands.Drive.ArcadeDrive;
import frc.robot.commands.Drive.FeedForwardCharacterization;
import frc.robot.commands.Drive.FeedForwardCharacterization1;
import frc.robot.commands.Drive.SetDistance;
import frc.robot.commands.Drive.TurnToAngle;
import frc.robot.commands.Elevator.AutoElevate;
import frc.robot.commands.Intake.AutonIntake;
import frc.robot.commands.Intake.FillerDefaultElevate;
import frc.robot.commands.Shooter.ActuateShooter;
import frc.robot.commands.Shooter.AlignShooter;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.DistanceSensor;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriveTrain2;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LightSensor;
import frc.robot.subsystems.Shooter;
import io.github.oblarg.oblog.Logger;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final DriveTrain driveTrain = new DriveTrain();
  // private final DriveTrain2 driveTrain = new DriveTrain2();
  
  private final Intake intake = new Intake();


  private final XboxController controller = new XboxController(Constants.Controller.PORT);
  private final XboxController joystick = new XboxController(Constants.Joystick.PORT);
  private final Shooter shooter = new Shooter();
  private final Climber climber = new Climber();
  
  private final DistanceSensor distanceSensor = new DistanceSensor();
  private final ColorSensor colorSensor = new ColorSensor();
  private final Vision vision = new Vision();
  private final LightSensor lightSensor = new LightSensor();

  private final Elevator elevator = new Elevator(intake, lightSensor, colorSensor);
  // SendableChooser<Pose2d> initialPosChooser = new SendableChooser<>();
  // SendableChooser<String> autonChooser = new SendableChooser<>();

  public RobotContainer() {
    Logger.configureLoggingAndConfig(this, false);
    
    
    // initialPosChooser.setDefaultOption("1", Constants.Field.INIT_POS_1);
    // initialPosChooser.addOption("2", Constants.Field.INIT_POS_2);
    // initialPosChooser.addOption("3", Constants.Field.INIT_POS_3);

    // autonChooser.setDefaultOption("Ball 1", "ball1");
    // autonChooser.addOption("Ball 2", "ball2");
    // autonChooser.addOption("Human Player", "humanplayer");

    // SmartDashboard.putData("Initial Position", initialPosChooser);
    // SmartDashboard.putData("Auton Sequence", autonChooser);

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // JoystickButton controllerbtn3 = new JoystickButton(controller, 3);
    // controllerbtn3.whenPressed(new InstantCommand(()->{
    //   driveTrain.zeroEncoders();
    // }));

    JoystickButton controllerBtn6 = new JoystickButton(controller, 6);
    controllerBtn6.whenPressed(new InstantCommand(()->{
      Constants.DriveTrain.MAX_VELOCITY = 3;
    })).whenReleased(new InstantCommand(()->{
      Constants.DriveTrain.MAX_VELOCITY = 2;
    }));

    JoystickButton controllerBtn1 = new JoystickButton(controller, 1);
    controllerBtn1.whenPressed(new InstantCommand(()->{
      Constants.DriveTrain.FORWARD_DIRECTION=1;
    }));

    JoystickButton controllerBtn2 = new JoystickButton(controller, 2);
    controllerBtn2.whenPressed(new InstantCommand(()->{
      Constants.DriveTrain.FORWARD_DIRECTION=-1;
    }));

    // JoystickButton controllerBtn4 = new JoystickButton(controller, 4);
    // controllerBtn4.whenPressed(new AlignShooter(driveTrain, vision));


    JoystickButton btn9 = new JoystickButton(joystick, 9);
    btn9.whenPressed(new InstantCommand(()->{
      elevator.setDefaultCommand(new AutoElevate(lightSensor, colorSensor, elevator, intake,0));
      Constants.Elevator.auto = true;
    }, elevator));

    JoystickButton btn8 = new JoystickButton(joystick, 8);
    btn8.whenPressed(new InstantCommand(()->{
      elevator.setDefaultCommand(new FillerDefaultElevate(elevator));
      Constants.Elevator.auto = false;
    }, elevator));

    JoystickButton btn11 = new JoystickButton(joystick, 11);
    btn11.whenPressed(new InstantCommand(()->{
      climber.setSpeed(0.5);
    }, climber)).whenReleased(new InstantCommand(()->{
      climber.stop();
    }, climber));

    JoystickButton btn10 = new JoystickButton(joystick, 10);
    btn10.whenPressed(new InstantCommand(()->{
      climber.setSpeed(-0.89);
    }, climber)).whenReleased(new InstantCommand(()->{
      climber.stop();
    }, climber));


    JoystickButton btn1 = new JoystickButton(joystick, 1);
    btn1.whenPressed(new InstantCommand(()->{
      intake.extend();
      intake.spin();
      if(!Constants.Elevator.auto){
      elevator.elevate();
      }
    }, intake)).whenReleased(new InstantCommand(()->{
      intake.retract();
      intake.stop();
      if(!Constants.Elevator.auto){
        elevator.stop();
        }
    }, intake));

    

    JoystickButton btn5 = new JoystickButton(joystick, 5);
    btn5.whenPressed(new InstantCommand(()->{
      intake.extend();
      intake.reverse();
      elevator.reverse();
    }, intake)).whenReleased(new InstantCommand(()->{
      intake.retract();
      intake.stop();
      elevator.stop();
    }, intake));

    JoystickButton btn3 = new JoystickButton(joystick, 3);
    btn3.whileHeld(new InstantCommand(()->{
      elevator.elevate();
    }, elevator)).whenReleased(new InstantCommand(()->{
      elevator.stop();
    },elevator));

    JoystickButton btn2 = new JoystickButton(joystick, 2);
    btn2.whileHeld(new InstantCommand(()->{
      elevator.reverse();
    })).whenReleased(new InstantCommand(()->{
      elevator.stop();
    }));

    JoystickButton btn6 = new JoystickButton(joystick, 6);
    btn6.whenPressed(new ActuateShooter(shooter, 0.15, 0.15,true, true));

    JoystickButton btn7 = new JoystickButton(joystick, 7);
    btn7.whenPressed(new ActuateShooter(shooter, 0.15, 
    0.15,true, false));

    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
  //   Pose2d initPos = initialPosChooser.getSelected();
  //   String autonName = autonChooser.getSelected();
    Command forward = new PreparedAuton(driveTrain, "paths/Forward.wpilib.json");
    Command backward = new PreparedAuton(driveTrain, "paths/Backward.wpilib.json");
    // Command auton1 = new PreparedAuton(driveTrain, "paths/startToShoot.wpilib.json");


  //   Command reverseAuton = new InstantCommand();


    InstantCommand setInitPos = new InstantCommand(()->{
      driveTrain.setFieldPos(Constants.Field.INIT_POS_2);
    }, driveTrain);

    ActuateShooter shoot = new ActuateShooter(shooter, 0.15, 0.15,true, true);
    ActuateShooter shoot2 = new ActuateShooter(shooter, 0.15, 0.15,true, true);

    AutonIntake startIntake = new AutonIntake(intake, elevator, true);

    AutonIntake stopIntake = new AutonIntake(intake, elevator, false);


  //   if(autonName.equals("ball1")){
  //     auton = new PreparedAuton(driveTrain, "paths/ball1.wpilib.json");
  //     reverseAuton = new PreparedAuton(driveTrain, "paths/ball1ToShoot.wpilib.json");
  //   }else if(autonName.equals("ball2")){
  //     auton = new PreparedAuton(driveTrain, "paths/ball2.wpilib.json");
  //     reverseAuton = new PreparedAuton(driveTrain, "paths/ball2ToShoot.wpilib.json");
  //   }else if(autonName.equals("humanplayer")){
  //     auton = new PreparedAuton(driveTrain, "paths/humanplayer.wpilib.json");
  //     reverseAuton = new PreparedAuton(driveTrain, "paths/humanplayerToShoot.wpilib.json");
  //   }
    elevator.setDefaultCommand(new FillerDefaultElevate(elevator));
    Constants.Elevator.auto = false;
    return (new SequentialCommandGroup(setInitPos, shoot, startIntake, forward, backward, stopIntake, shoot2));
    // return null;
  }
}
