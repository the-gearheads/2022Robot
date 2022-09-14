// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.cameraserver.CameraServer;
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
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Auton.AutonDrive2;
import frc.robot.commands.Auton.AutonDrive3;
import frc.robot.commands.Auton.PreparedAuton;
import frc.robot.commands.Auton.ReadAuton;
import frc.robot.commands.Auton.Wait;
import frc.robot.commands.Auton.WriteAuton;
import frc.robot.commands.Climber.AutomatedHighClimb;
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
import frc.robot.subsystems.FillerSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDS;
import frc.robot.subsystems.LightSensor;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

// auton speeds : 4m/s top speeds | acc: 2.5 m/s/s
// 3 ball : 1m/s top speed | .5 m/s/s

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
// public class RobotContainer {
//   private final DriveTrain driveTrain = new DriveTrain();
//   // private final DriveTrain2 driveTrain = new DriveTrain2();
//   Vision vision = new Vision();
  
//   // private final Intake intake = new Intake();


//   // TODO 3/31: set right port and length of buffer. // params: PWM Port and Length of strip
//   private final LEDS leds = new LEDS(9, 71);

//   private final XboxController controller = new XboxController(Constants.Controller.PORT);
//   private final XboxController joystick = new XboxController(Constants.Joystick.PORT);
//   private final Shooter shooter = new Shooter(leds);
//   private final Climber climber = new Climber();
  
//   // private final DistanceSensor distanceSensor = new DistanceSensor();
//   private final ColorSensor colorSensor = new ColorSensor();
//   private final LightSensor lightSensor = new LightSensor();
//   // private final AutonChooser autonChooser = new AutonChooser();
//   private final Intake intake = new Intake();

//   private final Elevator elevator = new Elevator(intake, lightSensor, colorSensor);
//   public RobotContainer() {
//     // Logger.configureLoggingAndConfig(this, false);
//     // SmartDashboard.putNumber("X val", 100);
//     // SmartDashboard.putNumber("Y val", 0);
//     // SmartDashboard.putNumber("Angle", 0);
//     SmartDashboard.putNumber("Turn 1", 0);
//     SmartDashboard.putNumber("Drive 1", 0);
//     SmartDashboard.putNumber("Turn 2", 0);
//     SmartDashboard.putNumber("Drive 2", 0);
    
//     SmartDashboard.putNumber("Turn 1 Reverse", 0);
//     SmartDashboard.putNumber("Drive 1 Reverse", 0);
//     SmartDashboard.putNumber("Turn 2 Reverse", 0);
//     SmartDashboard.putNumber("Drive 2 Reverse", 0);

//     SmartDashboard.putNumber("5Ball Y", 0);
//     SmartDashboard.putNumber("5Ball X", 0);
//     SmartDashboard.putNumber("5Ball Rot", 0);
//     SmartDashboard.putBoolean("5Ball IsBackward", false);


//     SmartDashboard.putNumber("Velocity", 1);
//     SmartDashboard.putNumber("Acc", 0.5);

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
//     JoystickButton controllerBtn1 = new JoystickButton(controller, 1);
//     controllerBtn1.toggleWhenPressed(new InstantCommand(()->{
//       driveTrain.setFieldPos(new Pose2d(7.668997828753121,1.5791062893481724, new Rotation2d(-1.5707963267948966)));
//       PreparedAuton intake = new PreparedAuton(driveTrain, "Right-5-Start");
//       TurnToAngle turn = new TurnToAngle(driveTrain, new Rotation2d(-1.917500707738926 - 2 * Math.PI).getDegrees());
//       (new SequentialCommandGroup(intake, turn)).schedule();
//     }));

//     // JoystickButton controllerBtn4 = new JoystickButton(controller, 4);
//     // controllerBtn4.whenPressed(new InstantCommand(()->{
//     //   driveTrain.setFieldPos(new Pose2d(0,0,new Rotation2d(0)));
//     //   TurnToAngle turn1 = new TurnToAngle(driveTrain, SmartDashboard.getNumber("Turn 1", 0));
//     //   AutonDrive2 drive1 = new AutonDrive2(driveTrain, -SmartDashboard.getNumber("Drive 1", 0));

//     //   TurnToAngle turn2 = new TurnToAngle(driveTrain, SmartDashboard.getNumber("Turn 2", 0));
//     //   AutonDrive2 drive2 = new AutonDrive2(driveTrain, -SmartDashboard.getNumber("Drive 2", 0));
      
//     //   SequentialCommandGroup auton = new SequentialCommandGroup(turn1, drive1, turn2, drive2);
//     //   auton.schedule();
//     // }));

//     // JoystickButton controllerBtn4 = new JoystickButton(controller, 4);
//     // controllerBtn4.toggleWhenPressed(new WriteAuton(driveTrain));

//     // JoystickButton controllerBtn3 = new JoystickButton(controller, 3);
//     // controllerBtn3.toggleWhenPressed(new ReadAuton(driveTrain, false));

//     JoystickButton controllerBtn4 = new JoystickButton(controller, 4);
//     controllerBtn4.whenPressed(new InstantCommand(()->{
//       double x = SmartDashboard.getNumber("5Ball X", 0);
//       double y = SmartDashboard.getNumber("5Ball Y", 0);
//       double rot = SmartDashboard.getNumber("5Ball Rot", 0);
//       boolean isBackward = SmartDashboard.getBoolean("5Ball IsBackward", false);

//       AutonDrive3 auton = new AutonDrive3(driveTrain, x,y,rot, isBackward);
//       auton.schedule();
//     }));
    
//     // JoystickButton controllerBtn3 = new JoystickButton(controller, 3);
//     // controllerBtn3.whenPressed(new InstantCommand(()->{
//     //   driveTrain.setFieldPos(new Pose2d(0,0,new Rotation2d(0)));
//     //   TurnToAngle turn1 = new TurnToAngle(driveTrain, -SmartDashboard.getNumber("Turn 1 Reverse", 0));
//     //   AutonDrive2 drive1 = new AutonDrive2(driveTrain, SmartDashboard.getNumber("Drive 1 Reverse", 0));

//     //   TurnToAngle turn2 = new TurnToAngle(driveTrain, -SmartDashboard.getNumber("Turn 2 Reverse", 0));
//     //   AutonDrive2 drive2 = new AutonDrive2(driveTrain, SmartDashboard.getNumber("Drive 2 Reverse", 0));
      
//     //   SequentialCommandGroup auton = new SequentialCommandGroup(drive2, turn2, drive1, turn1);
//     //   auton.schedule();
//     // }));

//     JoystickButton controllerBtn3 = new JoystickButton(controller, 3);
//     controllerBtn3.whenPressed(new InstantCommand(()->{
//       driveTrain.setFieldPos(new Pose2d(0,0, new Rotation2d(0)));
//     }));

//     JoystickButton controllerBtn2 = new JoystickButton(controller, 2);
//     controllerBtn2.whenPressed(new InstantCommand(()->{
//       driveTrain.setFieldPos(new Pose2d(7.668997828753121,1.5791062893481724, new Rotation2d(-1.5707963267948966)));

//       PreparedAuton intakeAuton = new PreparedAuton(driveTrain, "Right-5-Start");
//       TurnToAngle turn = new TurnToAngle(driveTrain, new Rotation2d(-1.917500707738926 - 2 * Math.PI).getDegrees());

//       AutonDrive3 toHuman = new AutonDrive3(driveTrain, 1.835, 0.16, -2.2, false);

//       AutonDrive3 toShoot = new AutonDrive3(driveTrain, 6.43, 1.695, -1.82, true);


//       InstantCommand zero1 = new InstantCommand(()->{
//         driveTrain.setFieldPos(new Pose2d(0,0,new Rotation2d(0)));
//       });

//       // TurnToAngle turn1 = new TurnToAngle(driveTrain, -49);
//       // AutonDrive2 drive1 = new AutonDrive2(driveTrain, -220);

//       // TurnToAngle turn2 = new TurnToAngle(driveTrain, 35);
//       // AutonDrive2 drive2 = new AutonDrive2(driveTrain, -15);

//       InstantCommand zero2 = new InstantCommand(()->{
//         driveTrain.setFieldPos(new Pose2d(0,0,new Rotation2d(0)));
//       });

//       // TurnToAngle turn1R = new TurnToAngle(driveTrain, 52);
//       // AutonDrive2 drive1R = new AutonDrive2(driveTrain, 210);

//       // TurnToAngle turn2R = new TurnToAngle(driveTrain, -42);
//       // AutonDrive2 drive2R = new AutonDrive2(driveTrain, 15);


//       InstantCommand startIntake = new InstantCommand(()->{
//         intake.extend();
//         intake.spin();
//       });
//       InstantCommand stopIntake = new InstantCommand(()->{
//         intake.retract();
//         intake.stop();
//       });

//       ActuateShooter shoot1 = new ActuateShooter(shooter, 0.15, 0.15,true, true);
//       ActuateShooter shoot2 = new ActuateShooter(shooter, 0.15, 0.15,true, true);
//       ActuateShooter shoot3 = new ActuateShooter(shooter, 0.15, 0.15,true, true);
//       ActuateShooter shoot4 = new ActuateShooter(shooter, 0.15, 0.15,true, true);
//       ActuateShooter shoot5 = new ActuateShooter(shooter, 0.15, 0.15,true, true);

//       Wait wait1 = new Wait(0.5);

//       Wait wait2 = new Wait(0.5);
//       Wait wait3 = new Wait(1);

//       elevator.setDefaultCommand(new AutoElevate(lightSensor, colorSensor, elevator, intake, 0));
//       // SequentialCommandGroup auton = new SequentialCommandGroup(startIntake, shoot1, intakeAuton, turn, shoot2, wait1, shoot3, toHuman, toShoot, shoot4, wait2, shoot5, stopIntake);
//       SequentialCommandGroup auton = new SequentialCommandGroup(intakeAuton, turn, toHuman, toShoot);
//       auton.schedule();
//     }));

    
//     JoystickButton btn8 = new JoystickButton(joystick, 8);
//     btn8.whenPressed(new InstantCommand(()->{
//       climber.liftArms();
//     }, climber));

//     JoystickButton btn9 = new JoystickButton(joystick, 9);
//     btn9.whenPressed(new InstantCommand(()->{
//       climber.lowerArms();
//     }, climber));

    
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
    


//     // JoystickButton controllerBtn3 = new JoystickButton(controller, 3);
//     // controllerBtn3.toggleWhenPressed(new AlignShooter(driveTrain));
//   }

//   /**
//    * Use this to pass the autonomous command to the main {@link Robot} class.
//    *
//    * @return the command to run in autonomous
//    */
//   public Command getAutonomousCommand() {
//     return null;
//     }
// }


//THIS IS A BARRIER++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

public class RobotContainer {
  private final DriveTrain driveTrain = new DriveTrain();
  // private final DriveTrain2 driveTrain = new DriveTrain2();
  
  private final Intake intake = new Intake();


  // TODO 3/31: set right port and length of buffer. // params: PWM Port and Length of strip
  private final LEDS leds = new LEDS(9, 71);

  private final XboxController controller = new XboxController(Constants.Controller.PORT);
  private final XboxController joystick = new XboxController(Constants.Joystick.PORT);
  private final Shooter shooter = new Shooter(leds);
  private final Climber climber = new Climber();
  
  private final DistanceSensor distanceSensor = new DistanceSensor();
  private final ColorSensor colorSensor = new ColorSensor();
  private final LightSensor lightSensor = new LightSensor();
  private final AutonChooser autonChooser = new AutonChooser();
  
  // private final Vision vision = new Vision(); //TURN ON FOR VISION

  private final FillerSubsystem fillerSubsystem = new FillerSubsystem();
  private final Elevator elevator = new Elevator(intake, lightSensor, colorSensor);
  public RobotContainer() {
    // Logger.configureLoggingAndConfig(this, false);
    // SmartDashboard.putNumber("5 Ball Human X",1.835);
    // SmartDashboard.putNumber("5 Ball Human Y", 0.16);
    // SmartDashboard.putNumber("5 Ball Human Rot", -2.2);
    SmartDashboard.putString("message", "nothing");


    // SmartDashboard.putNumber("5 Ball Shoot X", 6.43);
    // SmartDashboard.putNumber("5 Ball Shoot Y", 1.695);
    // SmartDashboard.putNumber("5 Ball Shoot Rot", -1.82);

    configureButtonBindings();

    PortForwarder.add(5800, "10.11.89.5", 80);
    PortForwarder.add(5801, "10.11.89.5", 1181);
    PortForwarder.add(5802, "10.11.89.5", 1182);

    SmartDashboard.putBoolean("4 Ball?", true);

    CameraServer.startAutomaticCapture().setResolution(640, 480);
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
    //JoystickButton controllerBtn4 = new JoystickButton(controller, 4);
    //controllerBtn4.toggleWhenPressed(new AlignShooter(driveTrain, leds));

    // JoystickButton controllerBtn3 = new JoystickButton(controller, 3);
    // controllerBtn3.toggleWhenPressed(new SetGreen(leds));

    // JoystickButton controllerBtn6 = new JoystickButton(controller, 6);
    // controllerBtn6.whenPressed(new InstantCommand(()->{
    //   Constants.DriveTrain.MAX_VELOCITY = 5;
    // })).whenReleased(new InstantCommand(()->{
    //   Constants.DriveTrain.MAX_VELOCITY = 2;
    // }));

    JoystickButton controllerBtn5 = new JoystickButton(controller, XboxController.Button.kRightBumper.value);
    controllerBtn5.whenPressed(new InstantCommand(()->{
      Constants.DriveTrain.FORWARD_DIRECTION=1;
    }));

    JoystickButton controllerBtn6 = new JoystickButton(controller, XboxController.Button.kRightBumper.value);
    controllerBtn6.whenPressed(new InstantCommand(()->{
      Constants.DriveTrain.FORWARD_DIRECTION=-1;
    }));

    // TURN ON FOR VISION
    // JoystickButton controllerBtn1 = new JoystickButton(controller, XboxController.Button.kA.value);
    // controllerBtn6.whenPressed(new AlignShooter(driveTrain, vision, leds)); 

    // JoystickButton controllerBtn1 = new JoystickButton(controller, 1);
    // controllerBtn1.whenPressed(new InstantCommand(()->{
    //   driveTrain.setRampRate(true);
    // }));

    // JoystickButton controllerBtn4 = new JoystickButton(controller, 4);
    // controllerBtn4.toggleWhenPressed(new StartEndCommand(()->{
    //   // driveTrain.setFieldPos(new Pose2d(7.668997828753121,1.5791062893481724, new Rotation2d(-1.5707963267948966)));
    //   // driveTrain.setFieldPos(new Pose2d(7.958396869730094,1.5205556378470781, new Rotation2d(-1.5474407682200622)));
    //   driveTrain.setFieldPos(new Pose2d(7.694735697315309,1.7842168102618619, new Rotation2d(-1.7818896600176435)));

    //   PreparedAuton intakeAuton = new PreparedAuton(driveTrain, "Right-5-Start");
    //   TurnToAngle turn = new TurnToAngle(driveTrain, new Rotation2d(-1.917500707738926 - 2 * Math.PI).getDegrees());

    //   AutonDrive3 toHuman = new AutonDrive3(driveTrain, SmartDashboard.getNumber("5 Ball Human X",1.835), SmartDashboard.getNumber("5 Ball Human Y", 0.16), SmartDashboard.getNumber("5 Ball Human Rot", -2.2), false);

    //   AutonDrive3 toShoot = new AutonDrive3(driveTrain, SmartDashboard.getNumber("5 Ball Shoot X", 6.43), SmartDashboard.getNumber("5 Ball Shoot Y", 1.695), SmartDashboard.getNumber("5 Ball Shoot Rot", -1.82), true);


    //   InstantCommand zero1 = new InstantCommand(()->{
    //     driveTrain.setFieldPos(new Pose2d(0,0,new Rotation2d(0)));
    //   });

    //   // TurnToAngle turn1 = new TurnToAngle(driveTrain, -49);
    //   // AutonDrive2 drive1 = new AutonDrive2(driveTrain, -220);

    //   // TurnToAngle turn2 = new TurnToAngle(driveTrain, 35);
    //   // AutonDrive2 drive2 = new AutonDrive2(driveTrain, -15);

    //   InstantCommand zero2 = new InstantCommand(()->{
    //     driveTrain.setFieldPos(new Pose2d(0,0,new Rotation2d(0)));
    //   });

    //   // TurnToAngle turn1R = new TurnToAngle(driveTrain, 52);
    //   // AutonDrive2 drive1R = new AutonDrive2(driveTrain, 210);

    //   // TurnToAngle turn2R = new TurnToAngle(driveTrain, -42);
    //   // AutonDrive2 drive2R = new AutonDrive2(driveTrain, 15);


    //   InstantCommand startIntake = new InstantCommand(()->{
    //     intake.extend();
    //     intake.spin();
    //     elevator.elevate();
    //     (new AutoElevate(lightSensor, colorSensor, elevator, intake, 0)).schedule();
    //   });
    //   InstantCommand stopIntake = new InstantCommand(()->{
    //     intake.retract();
    //     intake.stop();
    //     elevator.stop();
    //     (new FillerDefaultElevate(elevator)).schedule();
    //   });

    //   ActuateShooter shoot1 = new ActuateShooter(shooter, 0.15, 0.15,true, true);
    //   ActuateShooter shoot2 = new ActuateShooter(shooter, 0.15, 0.15,true, true);
    //   ActuateShooter shoot3 = new ActuateShooter(shooter, 0.15, 0.15,true, true);
    //   ActuateShooter shoot4 = new ActuateShooter(shooter, 0.15, 0.15,true, true);
    //   ActuateShooter shoot5 = new ActuateShooter(shooter, 0.15, 0.15,true, true);

    //   Wait wait1 = new Wait(0.5);

    //   Wait wait2 = new Wait(0.5);
    //   Wait wait3 = new Wait(1);
    //   // SequentialCommandGroup auton = new SequentialCommandGroup(startIntake, shoot1, intakeAuton, turn, shoot2, wait1, shoot3, toHuman, toShoot, shoot4, wait2, shoot5, stopIntake);
    //   SequentialCommandGroup auton = new SequentialCommandGroup(intakeAuton, turn, toHuman, toShoot);
    //   auton.addRequirements(fillerSubsystem);
    //   auton.schedule();
    // },
    // ()->{
    //   InstantCommand endAuton = new InstantCommand(()->{}, fillerSubsystem);
    //   endAuton.schedule();
    // }));

    // JoystickButton controllerBtn4 = new JoystickButton(controller, 4);
    // controllerBtn4.whenPressed(new AlignShooter(driveTrain, vision));

    JoystickButton btn8 = new JoystickButton(joystick, 8);
    btn8.whenPressed(new InstantCommand(()->{
      climber.liftArms();
    }, climber));

    JoystickButton btn9 = new JoystickButton(joystick, 9);
    btn9.whenPressed(new InstantCommand(()->{
      climber.lowerArms();
    }, climber));

    // JoystickButton btn4 = new JoystickButton(joystick, 4);
    // btn4.whenPressed(new InstantCommand(()->{
    //   (new FillerDefaultElevate(elevator)).schedule();
    //   Constants.Elevator.auto = false;
    // }, climber));
    
    JoystickButton btn4 = new JoystickButton(joystick, 4);
    btn4.toggleWhenPressed(new StartEndCommand(()->{
      (new AutoElevate(lightSensor, colorSensor, elevator, intake,0)).schedule();
      Constants.Elevator.auto = true;
    }, ()->{
      (new InstantCommand(()->{}, elevator)).schedule();
      Constants.Elevator.auto = false;
    }));

    // JoystickButton btn8 = new JoystickButton(joystick, 8);
    // btn8.whenPressed(new InstantCommand(()->{
    //   elevator.setDefaultCommand(new FillerDefaultElevate(elevator));
    //   Constants.Elevator.auto = false;
    // }, elevator));

    JoystickButton btn11 = new JoystickButton(joystick, 11);
    btn11.whileHeld(new InstantCommand(()->{
      climber.setSpeed(0.9);
    }, climber)).whenReleased(new InstantCommand(()->{
      climber.stop();
    }, climber));

    JoystickButton btn10 = new JoystickButton(joystick, 10);
    btn10.whileHeld(new InstantCommand(()->{
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

    // JoystickButton btn1  = new JoystickButton(joystick, 1);
    // btn1.whenPressed(new AutomatedHighClimb(climber));
    

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
    btn7.whenPressed(new ActuateShooter(shooter, 0.15, 0.15,true, false));

  //   JoystickButton btn7 = new JoystickButton(joystick, 7);
  //   btn7.whileHeld(new InstantCommand(()->{
  //     climber.setSpeed(-0.1);
  //   }, climber)).whenReleased(new InstantCommand(()->{
  //     climber.stop();
  //   }, climber));
  // }
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    if(!SmartDashboard.getBoolean("4 Ball?", false)){
      // if(false){
      InstantCommand setInitPos = new InstantCommand(()->{
        driveTrain.setFieldPos(Constants.Field.ZERO);
      }, driveTrain);
      PreparedAuton forward = new PreparedAuton(driveTrain, "Forward");
      PreparedAuton backward = new PreparedAuton(driveTrain, "Backward");
      ActuateShooter shoot1 = new ActuateShooter(shooter, 0.15, 0.15,true, true);
      ActuateShooter shoot2 = new ActuateShooter(shooter, 0.15, 0.15,true, true);
      InstantCommand startIntake = new InstantCommand(()->{
        intake.extend();
        intake.spin();
        elevator.elevate();
      });
      InstantCommand stopIntake = new InstantCommand(()->{
        intake.retract();
        intake.stop();
        elevator.stop();
      });
      return (new SequentialCommandGroup(setInitPos, shoot1, startIntake, forward, backward, shoot2, stopIntake));
    }else{
  //     return (new InstantCommand(()->{
  //       // InstantCommand zero1 = new InstantCommand(()->{
  //       //   driveTrain.setFieldPos(new Pose2d(0,0,new Rotation2d(0)));
  //       // });
  
  //       // TurnToAngle turn1 = new TurnToAngle(driveTrain, -49);
  //       // AutonDrive2 drive1 = new AutonDrive2(driveTrain, -220);
  
  //       // TurnToAngle turn2 = new TurnToAngle(driveTrain, 35);
  //       // AutonDrive2 drive2 = new AutonDrive2(driveTrain, -15);
  
  //       // InstantCommand zero2 = new InstantCommand(()->{
  //       //   driveTrain.setFieldPos(new Pose2d(0,0,new Rotation2d(0)));
  //       // });
  
  //       // TurnToAngle turn1R = new TurnToAngle(driveTrain, 52);
  //       // AutonDrive2 drive1R = new AutonDrive2(driveTrain, 210);
  
  //       // TurnToAngle turn2R = new TurnToAngle(driveTrain, -42);
  //       // AutonDrive2 drive2R = new AutonDrive2(driveTrain, 15);
  
  //       // driveTrain.setFieldPos(new Pose2d(7.668997828753121,1.5791062893481724, new Rotation2d(-1.5707963267948966)));
  //       // driveTrain.setFieldPos(new Pose2d(7.958396869730094,1.5205556378470781, new Rotation2d(-1.5474407682200622)));
  //       driveTrain.setFieldPos(new Pose2d(7.694735697315309,1.7842168102618619, new Rotation2d(-1.7818896600176435)));

  //       PreparedAuton intakeAuton = new PreparedAuton(driveTrain, "Right-5-Start");
  //       TurnToAngle turn = new TurnToAngle(driveTrain, new Rotation2d(-1.917500707738926 - 2 * Math.PI).getDegrees());
  
  //       AutonDrive3 toHuman = new AutonDrive3(driveTrain, 1.835, 0.16, -2.2, false);
  
  //       AutonDrive3 toShoot = new AutonDrive3(driveTrain, 6.43, 1.695, -1.82, true);
  

  //       InstantCommand startIntake = new InstantCommand(()->{
  //         intake.extend();
  //         intake.spin();
  //         (new AutoElevate(lightSensor, colorSensor, elevator, intake, 0)).schedule();
  //         Constants.Elevator.auto = true;
  //       });
  //       InstantCommand stopIntake = new InstantCommand(()->{
  //         intake.retract();
  //         intake.stop();
  //         (new FillerDefaultElevate(elevator)).schedule();
  //         Constants.Elevator.auto = false;
  //       });
  
  //       ActuateShooter shoot1 = new ActuateShooter(shooter, 0.15, 0.15,true, true);
  //       ActuateShooter shoot2 = new ActuateShooter(shooter, 0.15, 0.15,true, true);
  //       ActuateShooter shoot3 = new ActuateShooter(shooter, 0.15, 0.15,true, true);
  //       ActuateShooter shoot4 = new ActuateShooter(shooter, 0.15, 0.15,true, true);
  //       ActuateShooter shoot5 = new ActuateShooter(shooter, 0.15, 0.15,true, true);
  
  //       Wait wait1 = new Wait(0.5);
  
  //       Wait wait2 = new Wait(0.5);
  //       Wait wait3 = new Wait(1);
  //       SequentialCommandGroup auton = new SequentialCommandGroup(shoot1, startIntake, intakeAuton, turn, shoot2, wait1, shoot3, toHuman, toShoot, shoot4, wait2, shoot5, stopIntake);
  //       // SequentialCommandGroup auton = new SequentialCommandGroup(intakeAuton, turn, toHuman, toShoot);
  //       auton.schedule();
  //   }));
  // }
      InstantCommand setInitPos = new InstantCommand(()->{
        driveTrain.setFieldPos(Constants.Field.RIGHT_4_BALL);
      //  driveTrain.setRampRate(false);
      }, driveTrain);

      ActuateShooter shoot1 = new ActuateShooter(shooter, 0.15, 0.15,true, true);
      ActuateShooter shoot2 = new ActuateShooter(shooter, 0.15, 0.15,true, true);
      ActuateShooter shoot3 = new ActuateShooter(shooter, 0.15, 0.15,true, true);
      ActuateShooter shoot4 = new ActuateShooter(shooter, 0.15, 0.15,true, true);

      InstantCommand startIntake = new InstantCommand(()->{
        intake.extend();
        intake.spin();
        driveTrain.setRampRate(0);
        Constants.DriveTrain.MAX_VELOCITY = 2;
        Constants.DriveTrain.MAX_ROT_VELOCITY = 2.2;
        (new AutoElevate(lightSensor, colorSensor, elevator, intake, 0)).schedule();
        Constants.Elevator.auto = true;
      });
      InstantCommand stopIntake = new InstantCommand(()->{
        intake.retract();
        intake.stop();
        (new FillerDefaultElevate(elevator)).schedule();
        Constants.Elevator.auto = false;
       // driveTrain.setRampRate(true);
      });
      // PreparedAuton intakeTwo = new PreparedAuton(driveTrain, "Right-IntakeTwo");
      // PreparedAuton intakeTwoToShoot = new PreparedAuton(driveTrain, "Right-IntakeTwoToShoot")
      PreparedAuton startToIntake = new PreparedAuton(driveTrain, "Right-4-StartToIntake");
      PreparedAuton IntakeToShoot = new PreparedAuton(driveTrain, "Right-4-IntakeToShoot");
      PreparedAuton shootToHuman = new PreparedAuton(driveTrain, "Right-4-ShootToHuman");
      PreparedAuton humanToShoot = new PreparedAuton(driveTrain, "Right-4-HumanToShoot");

      TurnToAngle alignShot1 = new TurnToAngle(driveTrain, new Rotation2d(-2.1513027039072617).getDegrees());
      TurnToAngle alignShot2 = new TurnToAngle(driveTrain, new Rotation2d(-1.713027039072617).getDegrees());
      Wait wait1 = new Wait(1);
      Wait wait2 = new Wait(0.6);
      Wait wait3 = new Wait(0.5);
      Wait wait4 = new Wait(0.098765443212747427);
      return (new SequentialCommandGroup(setInitPos,shoot1, wait4, startIntake, startToIntake, IntakeToShoot, alignShot1,shoot2, shootToHuman, wait1, humanToShoot, wait3, shoot3, wait2, shoot4, stopIntake));
    }
    }
}
