// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriveTrainInterface;

public class TurnToAngle extends CommandBase {
  private double requestedAngle;
  private DriveTrainInterface driveTrain;
  private PIDController rotController = new PIDController(0.03, 0, 0);
  private int satisfactionNumber = 0;

  /** Creates a new TurnToAngleC. */
  public TurnToAngle(DriveTrainInterface driveTrain, double requestedAngle) {
    this.requestedAngle = requestedAngle;
    this.driveTrain = driveTrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Rotational Speed Calculations
    double currentAngle = driveTrain.getRotation();
    double maxRotSpeed = Constants.DriveTrain.MAX_ROT_VELOCITY * 2;
    double minRotSpeed = Constants.DriveTrain.MAX_ROT_VELOCITY / 4;

    double rotSpeed = rotController.calculate(currentAngle, requestedAngle);
    double rotDirection = rotSpeed > 0? -1 : 1;
    rotSpeed = Math.abs(rotSpeed);

    if(Math.abs(currentAngle - requestedAngle) < 3){
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
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double currentAngle = driveTrain.getRotation();
    if(Math.abs(currentAngle - requestedAngle) < 4){
      satisfactionNumber++;
    }else{
      satisfactionNumber=0;
    }
    return satisfactionNumber > 20;
  }
}
