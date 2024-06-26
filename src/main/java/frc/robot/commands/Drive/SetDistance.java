// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import javax.naming.spi.DirObjectFactory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriveTrainInterface;
import frc.robot.Constants;
import frc.robot.subsystems.DistanceSensor;

public class SetDistance extends Command {
  private DriveTrainInterface driveTrain;
  private DistanceSensor distanceSensor;
  private double requestedDistance;
  private PIDController linearController = new PIDController(0.03, 0, 0);
  private PIDController rotController = new PIDController(0.03, 0, 0);
  private double satisfactionNumber = 0;

  /** Creates a new SetDistance. */
  public SetDistance(DriveTrainInterface driveTrain, DistanceSensor distanceSensor, double requestedDistance) {
    this.driveTrain = driveTrain;
    this.distanceSensor = distanceSensor;
    this.requestedDistance = requestedDistance;
    addRequirements(driveTrain, distanceSensor);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.zeroEncoders();
    SmartDashboard.putBoolean("Is setDistance On", true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Linear Speed Calculations
    double maxSpeed = Constants.DriveTrain.MAX_VELOCITY;
    double minSpeed = 0.3;
    
    double currentDistance = distanceSensor.getDistanceInInches();
    double speed = linearController.calculate(currentDistance, requestedDistance);
    double direction = speed > 0? -1 : 1;
    speed = Math.abs(speed);

    if(Math.abs(currentDistance - requestedDistance) < 0.3){
      speed = 0;
    }else
    if(speed > maxSpeed){
      speed = maxSpeed;
    }else if(speed < minSpeed){
      speed = minSpeed;
    }

    //Rotational Speed Calculations
    double currentAngle = driveTrain.getRotation();
    double maxRotSpeed = Constants.DriveTrain.MAX_ROT_VELOCITY / 6;
    double minRotSpeed = 0.2 / 12;

    double rotSpeed = rotController.calculate(currentAngle, 0);
    double rotDirection = rotSpeed > 0? -1 : 1;
    rotSpeed = Math.abs(rotSpeed);

    if(Math.abs(currentAngle) < 1){
      rotSpeed = 0;
    }else
    if(Math.abs(rotSpeed) > maxRotSpeed){
      rotSpeed = maxRotSpeed;
    }else if(Math.abs(rotSpeed) < minRotSpeed){
      rotSpeed = minRotSpeed;
    }

    driveTrain.drive(speed * direction, rotSpeed * rotDirection);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.drive(0,0);
    SmartDashboard.putBoolean("Is setDistance On", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double currentDistance = distanceSensor.getDistanceInInches();
    double currentAngle = driveTrain.getRotation();
    if(Math.abs(currentDistance-requestedDistance) < 0.3 && Math.abs(currentAngle) < 1){
      satisfactionNumber++;
    }else{
      satisfactionNumber=0;
    }
    return satisfactionNumber > 20;
  }
}
