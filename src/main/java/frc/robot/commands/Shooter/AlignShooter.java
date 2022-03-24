// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainInterface;
import frc.robot.subsystems.Vision;

public class AlignShooter extends CommandBase {
  private double x;
  private PIDController pid = new PIDController(1.0, 0, 0);
  
  private double maxRotSpeed = 0.7;
  private double minRotSpeed = 0.1;
  private DriveTrainInterface driveTrain;
  private int satisfactionNumber = 0;
  private Vision vision;

  /** Creates a new AlignShooter. */
  public AlignShooter(DriveTrainInterface driveTrain, Vision vision) {
    this.driveTrain = driveTrain;
    addRequirements(driveTrain);
    this.vision = vision;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("Aligning", true);
    vision.setLED(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.x = SmartDashboard.getNumber("center X", 0);

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
}

// Called once the command ends or is interrupted.
@Override
public void end(boolean interrupted) {
  SmartDashboard.putBoolean("Aligning", false);
  vision.setLED(false);

}


// Returns true when the command should end.
@Override
public boolean isFinished() {
  if(Math.abs(x) < 0.05){
    satisfactionNumber++;
  }else{
    satisfactionNumber=0;
  }
  return satisfactionNumber > 20;
}
}
