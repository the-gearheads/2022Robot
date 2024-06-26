// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class SetIntakeSpeed extends Command {
  PIDController pid = new PIDController(1,0,0);
  Intake intake;
  double requestedSpeed;
  double cumulativeSpeed;
  /** Creates a new SetIntakeSpeed. */
  public SetIntakeSpeed(Intake intake, double requestedSpeed) {
    this.intake = intake;
    this.requestedSpeed = requestedSpeed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    cumulativeSpeed = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentSpeed = intake.getRPM();
    if(Math.abs(currentSpeed - requestedSpeed) > 0.1){
      double speedChange = pid.calculate(currentSpeed, requestedSpeed);
      cumulativeSpeed += speedChange;
    }
    intake.setSpeed(MathUtil.clamp(cumulativeSpeed, -1.0, 1.0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
