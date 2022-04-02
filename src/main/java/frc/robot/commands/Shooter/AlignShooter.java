// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.commands.LEDS.SetGreen;
import frc.robot.subsystems.DriveTrainInterface;
import frc.robot.subsystems.LEDS;

public class AlignShooter extends CommandBase {
  private double x;
  private PIDController pid = new PIDController(2, 0, 0);
  
  private double maxRotSpeed = 1.9;
  private double minRotSpeed = 0.8;
  private DriveTrainInterface driveTrain;
  private int satisfactionNumber = 0;
  private LEDS leds;
  private double nContours = 0;
  private boolean cannotAlign = false;
  private Timer checkTimer = new Timer();

  /** Creates a new AlignShooter. */
  public AlignShooter(DriveTrainInterface driveTrain, LEDS leds) {
    this.driveTrain = driveTrain;
    addRequirements(driveTrain);
    // Use addRequirements() here to declare subsystem dependencies.
    this.leds = leds;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //checkTimer.reset();
    //checkTimer.start();
    satisfactionNumber = 0;
    //nContours = SmartDashboard.getNumber("Contour Num", 0);

    // green leds
    (new SetGreen(leds)).schedule();
    
    /*if (nContours < 1) {
      (new SetAlignError(leds)).schedule(false);
      cannotAlign = true;
    }*/

    SmartDashboard.putBoolean("Aligning", true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  
  /*if (checkTimer.hasElapsed(0.25)) {
    if (nContours < 1) {
      (new SetAlignError(leds)).schedule(false);
      cannotAlign = true;
    }
  }*/  

  this.x = SmartDashboard.getNumber("Mean X", 0);
  
  double rotSpeed = pid.calculate(x, 0);
  double rotDirection = rotSpeed > 0? -1 : 1;
  rotSpeed = Math.abs(rotSpeed);

  if(Math.abs(x) < 0.05){
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

// Called once the command ends or is interrupted.
@Override
public void end(boolean interrupted) {
  SmartDashboard.putBoolean("Aligning", false);
  //checkTimer.stop();

  // return to rainbow leds
  leds.getDefaultCommand().schedule();
}


// Returns true when the command should end.
@Override
public boolean isFinished() {
  if(Math.abs(x) < 0.07){
    satisfactionNumber++;
  }else{
    satisfactionNumber=0;
  }
  return satisfactionNumber > 20 || cannotAlign;
}
}
