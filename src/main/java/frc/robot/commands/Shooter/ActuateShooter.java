// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.commands.LEDS.setPurpleLEDS;
import frc.robot.subsystems.LEDS;
import frc.robot.subsystems.Shooter;

public class ActuateShooter extends CommandBase {
  private Shooter shooter;
  private Timer timer = new Timer();
  private boolean isExtend;
  private boolean bothPistons;
  private double flowTime;
  private double totalTime;

  /** Creates a new Shoot. */
  public ActuateShooter(Shooter shooter, double flowTime, double totalTime, boolean isExtend, boolean bothPistons) {
    this.shooter = shooter;
    this.flowTime = flowTime;
    this.totalTime = totalTime;
    this.isExtend = isExtend;
    this.bothPistons = bothPistons;
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    if(isExtend){
      new setPurpleLEDS(shooter.leds);
      if(bothPistons){
        shooter.extend();
      }else{
        shooter.extendOne();
      }

      //External Code (sorry)
      if(Constants.Elevator.color_status){
        Constants.Elevator.shot = true;
      }
    }else{
      shooter.retract();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(timer.get() > flowTime && !shooter.isRest()){
      shooter.rest();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    if(isExtend){
      (new ActuateShooter(shooter, 0.1, 0.1,false, false)).schedule(false);
    }
  }
  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > totalTime;
  }
}
