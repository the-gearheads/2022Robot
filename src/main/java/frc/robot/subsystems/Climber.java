// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
    boolean disable = false;
    CANSparkMax left = new CANSparkMax(Constants.Climber.LEFT_ID, MotorType.kBrushless);
    CANSparkMax right = new CANSparkMax(Constants.Climber.RIGHT_ID, MotorType.kBrushless);
    RelativeEncoder rightEncoder = right.getEncoder();
    DigitalInput leftLimit = new DigitalInput(Constants.Climber.LEFT_LIMIT);
    DigitalInput rightLimit = new DigitalInput(Constants.Climber.RIGHT_LIMIT);
    private boolean rightDisable = false;
    private boolean leftDisable = false;

  public Climber() {
      left.setIdleMode(IdleMode.kBrake);
      right.setIdleMode(IdleMode.kBrake);
      rightEncoder.setPosition(0);
  }

  private double round(double value, double decimalPlaces){
    return ((int)(value * Math.pow(10, decimalPlaces))) / Math.pow(10,decimalPlaces);
  }

  public void setSpeed(double speed){
      if(!leftLimitTriggered() || speed > 0){
        left.set(speed);
      }else{
        left.set(0);
      }
      if(!rightLimitTriggered() || speed > 0){
        right.set(-speed);
      }else{
        right.set(0);
      }
  }

  public void stop(){
      setSpeed(0);
  }

  public boolean limitTriggered(){
      return !(leftLimit.get() && rightLimit.get());
  }

  public boolean rightLimitTriggered(){
    return !rightLimit.get();
  }

  public boolean leftLimitTriggered(){
    return !leftLimit.get();
  }

  @Override
  public void periodic() {
      SmartDashboard.putBoolean("LEFT Climber limit", !leftLimit.get());
      SmartDashboard.putBoolean("RIGHT Climber limit", !rightLimit.get());
  }

  public double getRotations(){
    return rightEncoder.getPosition();
  }
}
