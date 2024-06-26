// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class SetClimberPosition extends Command {
  private Climber climber;
  private double requestedPosition;
  private PIDController controller;
  private int satisfactionNumber;

  /** Creates a new SetClimberPosition. */
  public SetClimberPosition(Climber climber, double requestedPosition) {
    this.climber = climber;
    this.requestedPosition = requestedPosition;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    satisfactionNumber = 0;
  }

  public void execute() {
    //Rotational Speed Calculations
    double currentPosition = climber.getRotations();
    double maxSpeed = Constants.Climber.MAX_SPEED;
    double minSpeed = Constants.Climber.MIN_SPEED;

    double speed = controller.calculate(currentPosition, requestedPosition);
    double direction = speed > 0? -1 : 1;
    speed = Math.abs(speed);

    if(Math.abs(currentPosition - requestedPosition) < 1){
      speed = 0;
    }else if(Math.abs(speed) > maxSpeed){
      speed = maxSpeed;
    }else if(Math.abs(speed) < minSpeed){
      speed = minSpeed;
    }

    climber.setSpeed(speed * direction);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double currentAngle = climber.getRotations();
    if(Math.abs(currentAngle - requestedPosition) < 1){
      satisfactionNumber++;
    }else{
      satisfactionNumber=0;
    }
    return satisfactionNumber > 10;
  }
}
