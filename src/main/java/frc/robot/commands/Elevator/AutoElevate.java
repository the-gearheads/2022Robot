// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LightSensor;

public class AutoElevate extends CommandBase {
  /** Creates a new AutoElevate. */
  private double ballCount;

  private LightSensor lightSensor;
  private ColorSensor colorSensor;
  private Elevator elevator;
  
  private boolean prevLightStatus;
  private boolean prevIntake;


  private Intake intake;
  private Timer intakeTimer;
  private boolean intakeFlag;
  private double intakeTimeOut = 1;

  private boolean shouldStop;

  public AutoElevate(LightSensor lightSensor, ColorSensor colorSensor, Elevator elevator, Intake intake, int ballCount) {
    this.lightSensor = lightSensor;
    this.colorSensor = colorSensor;
    this.elevator = elevator;
    this.intake = intake;
    this.ballCount = ballCount;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    prevLightStatus = false;
    prevIntake = false;
    intakeTimer = new Timer();
    intakeTimer.reset();
    intakeTimer.stop();
    intakeFlag = false;
    shouldStop = false;
    Constants.Elevator.shot = false;
    SmartDashboard.putBoolean("Auto Elevate", true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    updateBallCount();
    updateIntakeFlag();
    setElevator();
    printValues();
  }
  private void printValues(){
    // SmartDashboard.putBoolean("intake Flag", intakeFlag);
    // SmartDashboard.putNumber("intake Timer", intakeTimer.get());
    // SmartDashboard.putBoolean("previous Light Status", prevLightStatus);
  }

  private void setElevator(){
    SmartDashboard.putString("Auto Elevate ERROR", "NONE");
    if(ballCount == 2){
      if(!colorSensor.redOrBlue().equals("unknown")){
        elevator.stop();
      }else{
        elevator.elevate();
      }
      SmartDashboard.putString("Auto Intake Status", "2 balls. We're full!");
    }else if(ballCount == 1){
      if(colorSensor.redOrBlue().equals("unknown") || intakeFlag){
        elevator.elevate();
        SmartDashboard.putString("Auto Intake Status", "1 ball and running! Either color sensor is false or intake is running.");
      }else{
        SmartDashboard.putString("Auto Intake Status", "1 ball and stopped. Both color sensor and intake are false.");
        elevator.stop();
      }
    }else if(ballCount==0){
      if(intakeFlag){
        elevator.elevate();
        SmartDashboard.putString("Auto Intake Status", "0 balls and running! Intake is on.");
      }else{
      elevator.stop();
      SmartDashboard.putString("Auto Intake Status", "0 balls and stopped. Intake is off.");
      }
    }else if(ballCount > 2 || ballCount < 0){
      SmartDashboard.putString("Auto Elevate ERROR", "ball count is not in allowed range");
      shouldStop = true;
    }
  }

  private void updateIntakeFlag(){
    boolean intakeOn = intake.getMotorSpeed() !=0;
    if(intakeOn){
      intakeFlag = true;
      intakeTimer.stop();
      intakeTimer.reset();
    }else{
       if(prevIntake){
          intakeTimer.start();
          intakeFlag = true;
       }else if(intakeTimer.get() > intakeTimeOut){
          intakeFlag = false;
       }
    }

    prevIntake = intakeOn;
  }

  private void updateBallCount(){
    boolean lightStatus = lightSensor.get();
    if(!prevLightStatus && lightStatus){
      ballCount++;
    }

    prevLightStatus = lightStatus;

    if(Constants.Elevator.shot){
      ballCount--;
      Constants.Elevator.shot = false;
    }

    Constants.Elevator.color_status = !colorSensor.redOrBlue().equals("unknown");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.stop();
    SmartDashboard.putBoolean("Auto Elevate", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shouldStop;
  }
}
