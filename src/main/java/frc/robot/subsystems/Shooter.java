// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.swing.text.DefaultStyledDocument.ElementSpec;

import edu.wpi.first.hal.REVPHFaults;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
  public final PneumaticHub hub = new PneumaticHub();
  boolean faults = false;
  public final Solenoid extendLeft = new Solenoid(PneumaticsModuleType.REVPH, Constants.Shooter.EXTEND_LEFT_SOLENOID);
  public final Solenoid retractLeft = new Solenoid(PneumaticsModuleType.REVPH, Constants.Shooter.RETRACT_LEFT_SOLENOID);
  public final Solenoid extendRight = new Solenoid(PneumaticsModuleType.REVPH, Constants.Shooter.EXTEND_RIGHT_SOLENOID);

  /** Creates a new ExampleSubsystem. */
  public Shooter() {
      rest();
  }

  public void extend(){
    retractLeft.set(false);
    extendLeft.set(true);
    extendRight.set(true);
  }

  public void extendOne(){
    retractLeft.set(false);
    extendLeft.set(false);
    extendRight.set(true);
  }

  public void retract(){
    extendLeft.set(false);
    extendRight.set(false);
    retractLeft.set(true);
  }
  
  public void rest(){
    extendLeft.set(false);
    extendRight.set(false);
    retractLeft.set(false);
  }

  public boolean isRest(){
    return !extendLeft.get() && !extendRight.get() && !retractLeft.get();
  }

  

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("Voltage", hub.getInputVoltage());    
    // SmartDashboard.putNumber("Current", hub.getCompressorCurrent()); 
    // REVPHFaults hubFault = hub.getFaults();
    // boolean isFault = hubFault.Brownout || hubFault.CanWarning || hubFault.CompressorOpen || hubFault.CompressorOverCurrent || hubFault.HardwareFault || hubFault.SolenoidOverCurrent;
    // if(isFault){
    //   faults = true;
    // }
    // SmartDashboard.putBoolean("faults", faults);
    // hubFault = hub.getFaults();   // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
