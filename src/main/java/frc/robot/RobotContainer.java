// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.net.PortForwarder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Auton.PreparedAuton;
import frc.robot.commands.Auton.ReadAuton;
import frc.robot.commands.Auton.Wait;
import frc.robot.commands.Auton.WriteAuton;
import frc.robot.commands.Drive.TurnToAngle;
import frc.robot.commands.Elevator.AutoElevate;
import frc.robot.commands.Intake.FillerDefaultElevate;
import frc.robot.commands.LEDS.SetGreen;
import frc.robot.commands.Shooter.ActuateShooter;
import frc.robot.commands.Shooter.AlignShooter;
import frc.robot.subsystems.AutonChooser;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.DistanceSensor;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriveTrain2;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDS;
import frc.robot.subsystems.LightSensor;
import frc.robot.subsystems.Shooter;

// auton speeds : 4m/s top speeds | acc: 2.5 m/s/s
// 3 ball : 1m/s top speed | .5 m/s/s

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // private final DriveTrain driveTrain = new DriveTrain();
  private final DriveTrain2 driveTrain = new DriveTrain2();
  
  // private final Intake intake = new Intake();


  // TODO 3/31: set right port and length of buffer. // params: PWM Port and Length of strip
  // private final LEDS leds = new LEDS(9, 71);

  private final XboxController controller = new XboxController(Constants.Controller.PORT);
  // private final XboxController joystick = new XboxController(Constants.Joystick.PORT);
  // private final Shooter shooter = new Shooter(leds);
  // private final Climber climber = new Climber();
  
  // private final DistanceSensor distanceSensor = new DistanceSensor();
  // private final ColorSensor colorSensor = new ColorSensor();
  // private final LightSensor lightSensor = new LightSensor();
  // private final AutonChooser autonChooser = new AutonChooser();

  // private final Elevator elevator = new Elevator(intake, lightSensor, colorSensor);

  public RobotContainer() {
    // Logger.configureLoggingAndConfig(this, false);

    configureButtonBindings();

    PortForwarder.add(5800, "10.11.89.5", 80);
    PortForwarder.add(5801, "10.11.89.5", 1181);
    PortForwarder.add(5802, "10.11.89.5", 1182);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    JoystickButton controllerBtn1 = new JoystickButton(controller, 1);
    controllerBtn1.toggleWhenPressed(new WriteAuton(driveTrain));

    JoystickButton controllerBtn4 = new JoystickButton(controller, 4);
    controllerBtn4.toggleWhenPressed(new ReadAuton(driveTrain, false));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
    }
}


// public class RobotContainer {
//   private final DriveTrain driveTrain = new DriveTrain();
//   // private final DriveTrain2 driveTrain = new DriveTrain2();
  
//   private final Intake intake = new Intake();


//   // TODO 3/31: set right port and length of buffer. // params: PWM Port and Length of strip
//   private final LEDS leds = new LEDS(9, 71);

//   private final XboxController controller = new XboxController(Constants.Controller.PORT);
//   private final XboxController joystick = new XboxController(Constants.Joystick.PORT);
//   private final Shooter shooter = new Shooter(leds);
//   private final Climber climber = new Climber();
  
//   private final DistanceSensor distanceSensor = new DistanceSensor();
//   private final ColorSensor colorSensor = new ColorSensor();
//   private final LightSensor lightSensor = new LightSensor();
//   private final AutonChooser autonChooser = new AutonChooser();

//   private final Elevator elevator = new Elevator(intake, lightSensor, colorSensor);

//   public RobotContainer() {
//     // Logger.configureLoggingAndConfig(this, false);

//     configureButtonBindings();

//     PortForwarder.add(5800, "10.11.89.5", 80);
//     PortForwarder.add(5801, "10.11.89.5", 1181);
//     PortForwarder.add(5802, "10.11.89.5", 1182);
//   }

//   /**
//    * Use this method to define your button->command mappings. Buttons can be created by
//    * instantiating a {@link GenericHID} or one of its subclasses ({@link
//    * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
//    * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
//    */
//   private void configureButtonBindings() {
//     // JoystickButton controllerbtn3 = new JoystickButton(controller, 3);
//     // controllerbtn3.whenPressed(new InstantCommand(()->{
//     //   driveTrain.zeroEncoders();
//     // }));
//     //JoystickButton controllerBtn4 = new JoystickButton(controller, 4);
//     //controllerBtn4.toggleWhenPressed(new AlignShooter(driveTrain, leds));

//     JoystickButton controllerBtn3 = new JoystickButton(controller, 3);
//     controllerBtn3.toggleWhenPressed(new SetGreen(leds));

//     JoystickButton controllerBtn6 = new JoystickButton(controller, 6);
//     controllerBtn6.whenPressed(new InstantCommand(()->{
//       Constants.DriveTrain.MAX_VELOCITY = 5;
//     })).whenReleased(new InstantCommand(()->{
//       Constants.DriveTrain.MAX_VELOCITY = 2;
//     }));

//     JoystickButton controllerBtn1 = new JoystickButton(controller, 1);
//     controllerBtn1.whenPressed(new InstantCommand(()->{
//       Constants.DriveTrain.FORWARD_DIRECTION=1;
//     }));

//     JoystickButton controllerBtn2 = new JoystickButton(controller, 2);
//     controllerBtn2.whenPressed(new InstantCommand(()->{
//       Constants.DriveTrain.FORWARD_DIRECTION=-1;
//     }));

//     // JoystickButton controllerBtn4 = new JoystickButton(controller, 4);
//     // controllerBtn4.whenPressed(new AlignShooter(driveTrain, vision));


//     JoystickButton btn9 = new JoystickButton(joystick, 9);
//     btn9.whenPressed(new InstantCommand(()->{
//       elevator.setDefaultCommand(new AutoElevate(lightSensor, colorSensor, elevator, intake,0));
//       Constants.Elevator.auto = true;
//     }, elevator));

//     JoystickButton btn8 = new JoystickButton(joystick, 8);
//     btn8.whenPressed(new InstantCommand(()->{
//       elevator.setDefaultCommand(new FillerDefaultElevate(elevator));
//       Constants.Elevator.auto = false;
//     }, elevator));

//     JoystickButton btn11 = new JoystickButton(joystick, 11);
//     btn11.whileHeld(new InstantCommand(()->{
//       climber.setSpeed(0.9);
//     }, climber)).whenReleased(new InstantCommand(()->{
//       climber.stop();
//     }, climber));

//     JoystickButton btn10 = new JoystickButton(joystick, 10);
//     btn10.whileHeld(new InstantCommand(()->{
//       climber.setSpeed(-0.89);
//     }, climber)).whenReleased(new InstantCommand(()->{
//       climber.stop();
//     }, climber));


//     JoystickButton btn1 = new JoystickButton(joystick, 1);
//     btn1.whenPressed(new InstantCommand(()->{
//       intake.extend();
//       intake.spin();
//       if(!Constants.Elevator.auto){
//       elevator.elevate();
//       }
//     }, intake)).whenReleased(new InstantCommand(()->{
//       intake.retract();
//       intake.stop();
//       if(!Constants.Elevator.auto){
//         elevator.stop();
//         }
//     }, intake));

    

//     JoystickButton btn5 = new JoystickButton(joystick, 5);
//     btn5.whenPressed(new InstantCommand(()->{
//       intake.extend();
//       intake.reverse();
//       elevator.reverse();
//     }, intake)).whenReleased(new InstantCommand(()->{
//       intake.retract();
//       intake.stop();
//       elevator.stop();
//     }, intake));

//     JoystickButton btn3 = new JoystickButton(joystick, 3);
//     btn3.whileHeld(new InstantCommand(()->{
//       elevator.elevate();
//     }, elevator)).whenReleased(new InstantCommand(()->{
//       elevator.stop();
//     },elevator));

//     JoystickButton btn2 = new JoystickButton(joystick, 2);
//     btn2.whileHeld(new InstantCommand(()->{
//       elevator.reverse();
//     })).whenReleased(new InstantCommand(()->{
//       elevator.stop();
//     }));

//     JoystickButton btn6 = new JoystickButton(joystick, 6);
//     btn6.whenPressed(new ActuateShooter(shooter, 0.15, 0.15,true, true));

//     JoystickButton btn7 = new JoystickButton(joystick, 7);
//     btn7.whenPressed(new ActuateShooter(shooter, 0.15, 
//     0.15,true, false));

    
//   }

//   /**
//    * Use this to pass the autonomous command to the main {@link Robot} class.
//    *
//    * @return the command to run in autonomous
//    */
//   public Command getAutonomousCommand() {
//     // if(!SmartDashboard.getBoolean("4 Ball?", false)){
//       InstantCommand setInitPos = new InstantCommand(()->{
//         driveTrain.setFieldPos(Constants.Field.ZERO);
//       }, driveTrain);
//       PreparedAuton forward = new PreparedAuton(driveTrain, "Forward");
//       PreparedAuton backward = new PreparedAuton(driveTrain, "Backward");
//       ActuateShooter shoot1 = new ActuateShooter(shooter, 0.15, 0.15,true, true);
//       ActuateShooter shoot2 = new ActuateShooter(shooter, 0.15, 0.15,true, true);
//       InstantCommand startIntake = new InstantCommand(()->{
//         intake.extend();
//         intake.spin();
//         elevator.elevate();
//       });
//       InstantCommand stopIntake = new InstantCommand(()->{
//         intake.retract();
//         intake.stop();
//         elevator.stop();
//       });
//       return (new SequentialCommandGroup(setInitPos, shoot1, startIntake, forward, backward, shoot2, stopIntake));
//     // }else{
//     //   InstantCommand setInitPos = new InstantCommand(()->{
//     //     driveTrain.setFieldPos(Constants.Field.RIGHT_4_BALL);
//     //   }, driveTrain);

//     //   ActuateShooter shoot1 = new ActuateShooter(shooter, 0.15, 0.15,true, true);
//     //   ActuateShooter shoot2 = new ActuateShooter(shooter, 0.15, 0.15,true, true);
//     //   ActuateShooter shoot3 = new ActuateShooter(shooter, 0.15, 0.15,true, true);
//     //   ActuateShooter shoot4 = new ActuateShooter(shooter, 0.15, 0.15,true, true);

//     //   InstantCommand startIntake = new InstantCommand(()->{
//     //     intake.extend();
//     //     intake.spin();
//     //   });
//     //   InstantCommand stopIntake = new InstantCommand(()->{
//     //     intake.retract();
//     //     intake.stop();
//     //   });
//     //   // PreparedAuton intakeTwo = new PreparedAuton(driveTrain, "Right-IntakeTwo");
//     //   // PreparedAuton intakeTwoToShoot = new PreparedAuton(driveTrain, "Right-IntakeTwoToShoot")
//     //   PreparedAuton startToIntake = new PreparedAuton(driveTrain, "Right-4-StartToIntake");
//     //   PreparedAuton IntakeToShoot = new PreparedAuton(driveTrain, "Right-4-IntakeToShoot");
//     //   PreparedAuton shootToHuman = new PreparedAuton(driveTrain, "Right-4-ShootToHuman");
//     //   PreparedAuton humanToShoot = new PreparedAuton(driveTrain, "Right-4-HumanToShoot");

//     //   TurnToAngle alignShot1 = new TurnToAngle(driveTrain, new Rotation2d(-2.1513027039072617).getDegrees());
//     //   TurnToAngle alignShot2 = new TurnToAngle(driveTrain, new Rotation2d(-1.713027039072617).getDegrees());
//     //   Wait wait1 = new Wait(1);
//     //   Wait wait2 = new Wait(1);
//     //   Wait wait3 = new Wait(1);

//     //   elevator.setDefaultCommand(new AutoElevate(lightSensor, colorSensor, elevator, intake, 0));
//     //   Constants.Elevator.auto = true;
//     //   return (new SequentialCommandGroup(setInitPos,shoot1, startIntake, startToIntake, IntakeToShoot, alignShot1,shoot2, shootToHuman, wait1,humanToShoot, shoot3, wait2, shoot4, stopIntake));
//     // }
//     }
// }
