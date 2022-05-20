// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class AutomatedHighClimb extends CommandBase {
  private Timer timer;
  private int stage;
  private Climber climber;
  private boolean secure;
  private int currentLimit;
  private boolean running;

  /** Creates a new AutomatedHighClimb. */
  public AutomatedHighClimb(Climber climber) {
    this.climber = climber;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.timer = null;
    stage = 1;
    this.secure = false;
    running = true;
  }

  public void print(){
    SmartDashboard.putNumber("Climber Stage", stage);
    SmartDashboard.putNumber("Left Climber Current", climber.getLeftCurrent());
    SmartDashboard.putNumber("Right Climber Current", climber.getRightCurrent());
    SmartDashboard.putBoolean("isRunning", running);
    SmartDashboard.putBoolean("secure", secure);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    print();
    switch (stage){
      case 1:
        if(!climber.getUpperRightLimit() || !climber.getUpperLeftLimit()){
          climber.setSpeed(0.9);
        }else{
          climber.setSpeed(0);
          stage = 2;
        }
        break;
      case 2:
        climber.lowerArms();
        break;
      case 3: 
        if(timer == null){
          timer = new Timer();
          timer.reset();
          timer.start();
        }
        else if(timer.get() < 2){
          climber.setSpeed(-0.1);
          if(climber.getRightCurrent() > currentLimit && climber.getLeftCurrent() > currentLimit){
            secure = true;
          }
        }else{
          if(secure == true){
            stage = 4;
          }else{
            stage = -1;
          }
        }
        break;
      case 4:
        if(!climber.getLowerRightLimit() || !climber.getLowerLeftLimit()){
          climber.setSpeed(-0.9);
        }else{
          climber.setSpeed(0);
          stage = 5;
        }
        break;
      case 5:
        climber.liftArms();
        stage = 6;
        break;
      case -1:
        running = false;
        break;
      default:
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.setSpeed(0);
    running = false;

    print();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return stage == 6 || running == false;
  }
}