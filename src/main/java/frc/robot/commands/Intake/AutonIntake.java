// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import javax.swing.text.DefaultStyledDocument.ElementSpec;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

public class AutonIntake extends CommandBase {
  private Intake intake;
  private Elevator elevator;
  private boolean start;
  /** Creates a new AutonIntake. */
  public AutonIntake(Intake intake, Elevator elevator, boolean start) {
    this.intake = intake;
    this.elevator = elevator;
    this.start = start;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(start){
      intake.extend();
      intake.spin();
      elevator.elevate();
    }else{
      intake.retract();
      intake.stop();
      elevator.stop();
    }
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
