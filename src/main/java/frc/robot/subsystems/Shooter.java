// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.swing.text.DefaultStyledDocument.ElementSpec;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    DoubleSolenoid firstPiston = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.Shooter.FIRST_PISTON_FORWARD, Constants.Shooter.FIRST_PISTON_BACKWARD);
    DoubleSolenoid secondPiston = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.Shooter.SECOND_PISTON_FORWARD, Constants.Shooter.SECOND_PISTON_BACKWARD);
  /** Creates a new ExampleSubsystem. */
  public Shooter() {
      rest();
  }

  public void extend(){
    setFirstPiston(Value.kForward);
    setSecondPiston(Value.kForward);
  }

  public void retract(){
    setFirstPiston(Value.kReverse);
    setSecondPiston(Value.kOff);
  }
  public void rest(){
      setFirstPiston(Value.kOff);
      setSecondPiston(Value.kOff);
  }

  public Value getStatus(){
    return firstPiston.get();
  }

  /**
   * Set the value of a solenoid.
   *
   * @param value The value to set (Off, Forward, Reverse)
   */
  public void setFirstPiston(Value value){
    secondPiston.set(value);
}

  /**
   * Set the value of a solenoid.
   *
   * @param value The value to set (Off, Forward, Reverse)
   */
  public void setSecondPiston(Value value){
    firstPiston.set(value);
  }

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
