// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrainInterface;

public class FeedForwardCharacterization extends Command {
  private DriveTrainInterface driveTrain;
  private double testSpeed;
  private double maxVoltage;
  private double kV;
  private double kS;
  private double kVIncrement;
  private double satisfactionNumber;
  private int count = 0;
  private double vel = 0;

  /** Creates a new FeedForwardCharacterization. */
  public FeedForwardCharacterization(DriveTrainInterface driveTrain, double testSpeed, double maxVoltage, double kS, double kVIncrement, double kVStart) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;
    this.testSpeed = testSpeed;
    this.maxVoltage = maxVoltage;
    this.kS = kS;
    this.kVIncrement = kVIncrement;
    this.kV = kVStart;

    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("Characterizing", true);
    SmartDashboard.putNumber("kS", kS);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    count++;
    if(count % 5 != 1){
      return;
    }else{
      vel += driveTrain.getLeftVelocity();
      SmartDashboard.putString("Measured Vels", "Right: " + driveTrain.getRightVelocity() + "; Left: " + driveTrain.getLeftVelocity());
      if(count % 25 != 1){
        return;
      }
      vel /= 5;
    }

    if(Math.abs(vel - testSpeed) < testSpeed / 10.0){
      satisfactionNumber+=0.5;
    }else{
      satisfactionNumber = 0;
      double increment = kVIncrement < Math.abs(testSpeed - vel) * 3? Math.abs(testSpeed-vel) * 3:kVIncrement;
      if(vel > testSpeed){
        kV-=increment;
      }else if(vel < testSpeed){
        kV+=increment;
      }
    }

    kS = SmartDashboard.getNumber("kS", kS);

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV);  

    double newVel = MathUtil.clamp(feedforward.calculate(testSpeed), -maxVoltage, maxVoltage);
    SmartDashboard.putNumber("Feedforward kV", kV);
    SmartDashboard.putNumber("Satisfaction Number", satisfactionNumber);

    driveTrain.setSpeeds(newVel, newVel);

    vel = 0;

    // driveTrain.printValues();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.setSpeeds(0,0);
    SmartDashboard.putBoolean("Characterizing", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return satisfactionNumber > 20;
  }
}
