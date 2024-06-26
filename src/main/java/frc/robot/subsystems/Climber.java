// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.Climber.DefaultClimber;

public class Climber extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
    boolean disable = false;
    CANSparkMax left = new CANSparkMax(Constants.Climber.LEFT_ID, MotorType.kBrushless);
    CANSparkMax right = new CANSparkMax(Constants.Climber.RIGHT_ID, MotorType.kBrushless);
    RelativeEncoder rightEncoder = right.getEncoder();
    DigitalInput lowerLeftLimit = new DigitalInput(Constants.Climber.LEFT_LIMIT);
    DigitalInput lowerRightLimit = new DigitalInput(Constants.Climber.RIGHT_LIMIT);

    DigitalInput upperLeftLimit = new DigitalInput(Constants.Climber.UPPER_LEFT_LIMIT);
    DigitalInput upperRightLimit = new DigitalInput(Constants.Climber.UPPER_RIGHT_LIMIT);
    LinearFilter leftCurrentFilter = LinearFilter.movingAverage(15);
    LinearFilter rightCurrentFilter = LinearFilter.movingAverage(15);

    private boolean rightDisable = false;
    private boolean leftDisable = false;

    public final Solenoid armsRetract = new Solenoid(PneumaticsModuleType.REVPH, Constants.Climber.ARMS_RETRACT);
    public final Solenoid armsExtend = new Solenoid(PneumaticsModuleType.REVPH, Constants.Climber.ARMS_EXTEND);

  public Climber() {
      left.setIdleMode(IdleMode.kBrake);
      right.setIdleMode(IdleMode.kBrake);
      rightEncoder.setPosition(0);

      setDefaultCommand(new DefaultClimber(this));

  }

  public void liftArms() {
    armsRetract.set(false);
    armsExtend.set(true);
  }

  public void lowerArms() {
    armsRetract.set(true);
    armsExtend.set(false);
  }

  public void restArms(){
    armsRetract.set(false);
    armsExtend.set(false);
  }

  public boolean areArmsAtRest(){
    return !armsRetract.get() && !armsExtend.get();
  }

  private double round(double value, double decimalPlaces){
    return ((int)(value * Math.pow(10, decimalPlaces))) / Math.pow(10,decimalPlaces);
  }

  public void setSpeed(double speed){
      if((!getLowerLeftLimit() || speed >= 0) && (!getUpperLeftLimit() || speed <= 0)){
        left.set(speed);
      }else{
        left.set(0);
      }
      if((!getLowerRightLimit() || speed >= 0) && (!getUpperRightLimit() || speed <= 0)){
        right.set(-speed);
      }else{
        right.set(0);
      }

  }

  public void stop(){
      setSpeed(0);
  }

  public boolean limitTriggered(){
      return !(lowerLeftLimit.get() && lowerRightLimit.get());
  }

  public boolean getLowerRightLimit(){
    return !lowerRightLimit.get();
  }

  public boolean getLowerLeftLimit(){
    return !lowerLeftLimit.get();
  }

  @Override
  public void periodic() {
      SmartDashboard.putBoolean("LOWER LEFT Climber limit", !lowerLeftLimit.get());
      SmartDashboard.putBoolean("LOWER RIGHT Climber limit", !lowerRightLimit.get());
      SmartDashboard.putBoolean("UPPER LEFT Climber limit", !upperLeftLimit.get());
      SmartDashboard.putBoolean("UPPER RIGHT Climber limit", !upperRightLimit.get());
      SmartDashboard.putBoolean("UPPER RIGHT Climber limit", !upperRightLimit.get());
      SmartDashboard.putNumber("Climber Right Current", getRightCurrent());
      SmartDashboard.putNumber("Climber Left Current", getLeftCurrent());
      leftCurrentFilter.calculate(left.getOutputCurrent());
      rightCurrentFilter.calculate(right.getOutputCurrent());

  }

  public double getRotations(){
    return rightEncoder.getPosition();
  }

public boolean getUpperRightLimit() {
    return !upperRightLimit.get();
}

public boolean getUpperLeftLimit() {
    return !upperLeftLimit.get();
}

public double getRightCurrent() {
    return rightCurrentFilter.calculate(right.getOutputCurrent());
}

public double getLeftCurrent() {
  return leftCurrentFilter.calculate(left.getOutputCurrent());
}
}
