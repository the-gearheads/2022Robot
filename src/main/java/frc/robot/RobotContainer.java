// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Auton.PreparedAuton;
import frc.robot.commands.Auton.Wait;
import frc.robot.commands.Drive.TurnToAngle;
import frc.robot.commands.Elevator.AutoElevate;
import frc.robot.commands.Intake.FillerDefaultElevate;
import frc.robot.commands.Shooter.ActuateShooter;
import frc.robot.subsystems.AutonChooser;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.DistanceSensor;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.FillerSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LEDS;
import frc.robot.subsystems.LightSensor;
import frc.robot.subsystems.Shooter;

//THIS IS A BARRIER++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

public class RobotContainer {
  private final DriveTrain driveTrain = new DriveTrain();
  // private final DriveTrain2 driveTrain = new DriveTrain2();
  
  private final Intake intake = new Intake();


  // TODO 3/31: set right port and length of buffer. // params: PWM Port and Length of strip
  private final LEDS leds = new LEDS(9, 71);

  private final XboxController controller = new XboxController(Constants.Controller.PORT);
  private final Shooter shooter = new Shooter(leds);
  private final Climber climber = new Climber();
  
  @SuppressWarnings("unused")
  private final DistanceSensor distanceSensor = new DistanceSensor();
  private final ColorSensor colorSensor = new ColorSensor();
  private final LightSensor lightSensor = new LightSensor();
  @SuppressWarnings("unused")
  private final AutonChooser autonChooser = new AutonChooser();
  
  // private final Vision vision = new Vision(); //TURN ON FOR VISION

  @SuppressWarnings("unused")
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

    Trigger speedBtn = new Trigger(controller::getLeftBumper);
    speedBtn.onTrue(new InstantCommand(()->{
      Constants.DriveTrain.MAX_VELOCITY = 5;
    })).onFalse(new InstantCommand(()->{
      Constants.DriveTrain.MAX_VELOCITY = 2;
    }));

    
    Trigger btn4 = new Trigger(controller::getRightBumper);
    btn4.toggleOnTrue(new StartEndCommand(()->{
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

    Trigger intakeBtn = new Trigger(() -> controller.getRightTriggerAxis() > 0.1);
    intakeBtn.whileTrue(new InstantCommand(()->{
      intake.extend();
      intake.spin();
      if(!Constants.Elevator.auto){
      elevator.elevate();
      }
    }, intake)).whileFalse(new InstantCommand(()->{
      intake.retract();
      intake.stop();
      if(!Constants.Elevator.auto){
        elevator.stop();
        }
    }, intake));

    // JoystickButton btn1  = new JoystickButton(joystick, 1);
    // btn1.whenPressed(new AutomatedHighClimb(climber));
    

    Trigger manualIntake = new Trigger(controller::getYButton);
    manualIntake.whileTrue(new InstantCommand(()->{ 
      intake.extend();
      intake.reverse();
      elevator.reverse();
    }, intake)).whileFalse(new InstantCommand(()->{
      intake.retract();
      intake.stop();
      elevator.stop();
    }, intake));

    Trigger manualElevator = new Trigger(() -> controller.getPOV() == 0);
    manualElevator.whileTrue(new InstantCommand(()->{
      elevator.elevate();
    }, elevator)).whileFalse(new InstantCommand(()->{
      elevator.stop();
    },elevator));

    Trigger reverseElevator = new Trigger(() -> controller.getPOV() == 180);
    reverseElevator.whileTrue(new InstantCommand(()->{
      elevator.reverse();
    })).whileFalse(new InstantCommand(()->{
      elevator.stop();
    }));

    Trigger fullShotBtn = new Trigger(controller::getXButton);
    fullShotBtn.whileTrue(new ActuateShooter(shooter, 0.15, 0.15,true, true));

    Trigger halfShotBtn = new Trigger(controller::getAButton);
    halfShotBtn.whileTrue(new ActuateShooter(shooter, 0.15, 0.15,true, false));

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
      Wait pleaseWait = new Wait(Math.PI/30);
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
      return (new SequentialCommandGroup(setInitPos, shoot1, startIntake, pleaseWait, forward, backward, shoot2, stopIntake));
    }else{

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
      @SuppressWarnings("unused")
      TurnToAngle alignShot2 = new TurnToAngle(driveTrain, new Rotation2d(-1.713027039072617).getDegrees());
      Wait wait1 = new Wait(1);
      Wait wait2 = new Wait(0.6);
      Wait wait3 = new Wait(0.5);
      Wait wait4 = new Wait(0.098765443212747427);
      return (new SequentialCommandGroup(setInitPos,shoot1, wait4, startIntake, startToIntake, IntakeToShoot, alignShot1,shoot2, shootToHuman, wait1, humanToShoot, wait3, shoot3, wait2, shoot4, stopIntake));
    }
    }
}
