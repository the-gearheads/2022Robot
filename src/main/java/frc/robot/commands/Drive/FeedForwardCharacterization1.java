// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import javax.naming.spi.DirObjectFactory;

import com.ctre.phoenix.led.ColorFlowAnimation.Direction;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainInterface;

public class FeedForwardCharacterization1 extends CommandBase {
  private DriveTrainInterface driveTrain;
  private double testSpeed;
  private double maxVoltage;
  private double leftkV = 0;
  private double rightkV = 0;
  private double kVIncrement;
  private double kS;
  private double satisfactionNumber;
  private int count = 0;
  private double leftVel = 0;
  private double rightVel = 0;

  /** Creates a new FeedForwardCharacterization. */
  public FeedForwardCharacterization1(DriveTrainInterface driveTrain, double testSpeed, double maxVoltage, double kS, double kVIncrement, double kVStart) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;
    this.testSpeed = testSpeed;
    this.maxVoltage = maxVoltage;
    this.kS = kS;
    this.kVIncrement = kVIncrement;
    leftkV = kVStart;
    rightkV = kVStart;

    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("Characterizing", true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    count++;
    if(count % 5 != 1){
      return;
    }else{
      leftVel += driveTrain.getLeftVelocity();
      rightVel += driveTrain.getRightVelocity();
      SmartDashboard.putString("Measured Vels", "Right: " + driveTrain.getRightVelocity() + "; Left: " + driveTrain.getLeftVelocity());
      if(count % 25 != 1){
        return;
      }
      leftVel/=5;
      rightVel/=5;
    }

    if(Math.abs(leftVel - testSpeed) < 0.02){
      satisfactionNumber+=0.5;
    }else{
      satisfactionNumber = 0;
      double lIncrement = kVIncrement < Math.abs(testSpeed - leftVel) * 3? Math.abs(testSpeed-leftVel) * 3:kVIncrement;
      if(leftVel > testSpeed){
        leftkV-=lIncrement;
      }else if(leftVel < testSpeed){
        leftkV+=lIncrement;
      }
    }

    if(Math.abs(rightVel - testSpeed) < 0.02){
      satisfactionNumber+=0.5;
    }else{
      satisfactionNumber = 0;
      double rIncrement = kVIncrement < Math.abs(testSpeed - rightVel) * 3? Math.abs(testSpeed-rightVel) * 3: kVIncrement;
      if(rightVel > testSpeed){
        rightkV-=rIncrement;
      }else if(rightVel < testSpeed){
        rightkV+=rIncrement;
      }
    }

    

    SimpleMotorFeedforward lFeedforward = new SimpleMotorFeedforward(kS, leftkV);
    SimpleMotorFeedforward rFeedforward = new SimpleMotorFeedforward(kS, rightkV);
    
    double newLeftVel = MathUtil.clamp(lFeedforward.calculate(testSpeed), -maxVoltage, maxVoltage);
    double newRightVel = MathUtil.clamp(rFeedforward.calculate(testSpeed), -maxVoltage, maxVoltage);

    SmartDashboard.putString("Feedforward kVs", "Right: " + rightkV + "; Left: " + leftkV);
    SmartDashboard.putString("Speeds", "Right: " + rightVel + "; Left: " + leftVel);
    SmartDashboard.putNumber("Satisfaction Number", satisfactionNumber);

    driveTrain.setSpeeds(newLeftVel, newRightVel);

    leftVel = 0;
    rightVel = 0;
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
