// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.commands.LEDS.SetGreen;
import frc.robot.subsystems.DriveTrainInterface;
import frc.robot.subsystems.LEDS;
import frc.robot.subsystems.Vision;

public class AlignShooter extends CommandBase {
  private PIDController pid = new PIDController(0.1, 0, 0);
  private double maxRotSpeed = 1.9;
  private double minRotSpeed = 0.8;
  private double x;
  
  private DriveTrainInterface driveTrain;
  private LEDS leds;
  private Vision vision;

  private int satisfactionNumber = 0;
  private boolean canAlign = false;
  private Timer timer = new Timer();

  /** Creates a new AlignShooter. */
  public AlignShooter(DriveTrainInterface driveTrain, LEDS leds, Vision vision) {
    this.driveTrain = driveTrain;
    this.leds = leds;
    this.vision = vision;
    addRequirements(driveTrain, vision, leds);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("Aligning", true);
    satisfactionNumber = 0;
    timer.reset();
    timer.start();
    
    vision.connectToCamera();
    this.canAlign = vision.isVisionWorking();

    leds.setRainbow(false);
    if(canAlign){
      leds.updateStrips(leds.greenBuffer);
    }else{
      leds.updateStrips(leds.orangeBuffer);
    }
    leds.startStrips();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(canAlign){
      x = vision.getTargetXPos();
      double rotSpeed = pid.calculate(x, 0);
      double rotDirection = rotSpeed > 0? 1 : -1;// CHANGE TO 1 : -1
      rotSpeed = Math.abs(rotSpeed);
    
      if(Math.abs(x) < 3){
        rotSpeed = 0;
      }else
      if(Math.abs(rotSpeed) > maxRotSpeed){
        rotSpeed = maxRotSpeed;
      }else if(Math.abs(rotSpeed) < minRotSpeed){
        rotSpeed = minRotSpeed;
      }
       
      driveTrain.drive(0, rotSpeed * rotDirection);
    
      SmartDashboard.putNumber("Satisfaction N.", satisfactionNumber);
    }
}

// Called once the command ends or is interrupted.
@Override
public void end(boolean interrupted) {
  SmartDashboard.putBoolean("Aligning", false);
  leds.setRainbow(true);
}


// Returns true when the command should end.
@Override
public boolean isFinished() {
  if(Math.abs(x) < 4){
    satisfactionNumber++;
  }else{
    satisfactionNumber=0;
  }
  boolean tookTooLong = false;
  if(timer.get() > 2){
    tookTooLong = true;
    timer.stop();
    SmartDashboard.putBoolean("Took Too Long To Align Shooter", true);
  }
  return satisfactionNumber > 20 || !canAlign || tookTooLong;
}
}
