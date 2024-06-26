// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auton;

import java.util.ArrayList;
import java.util.HashMap;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrainInterface;

public class CreatePath extends Command {
  private DriveTrainInterface driveTrain;
  private Timer timer = new Timer();
  private int status = 0;
  private ArrayList<HashMap<String,Double>> path = new ArrayList<HashMap<String,Double>>();
  private double prevSpeed = 0;

  /** Creates a new CreatePath. */
  public CreatePath(DriveTrainInterface driveTrain) {
    this.driveTrain = driveTrain;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    SmartDashboard.putBoolean("Collect path data", false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(SmartDashboard.getBoolean("Collect path data", false) && status == 0){
      status = 1;
    }else if(status == 1){
      if(!SmartDashboard.getBoolean("Collect path data", true)){
        status = 2;
        return;
      }

      HashMap<String, Double> data = new HashMap<String, Double>();
      data.put("acceleration", driveTrain.getLeftVelocity() - prevSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return status == 2;
  }
}
