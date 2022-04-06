// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.Intake.DefaultIntake;

public class Intake extends SubsystemBase {
  private final double speed = 0.6;
  private final CANSparkMax leftMotor = new CANSparkMax(Constants.Intake.LEFT_MOTOR, MotorType.kBrushless);
  private final CANSparkMax rightMotor = new CANSparkMax(Constants.Intake.RIGHT_MOTOR, MotorType.kBrushless);
  private final RelativeEncoder rightEncoder = rightMotor.getEncoder();
  public final Solenoid retractSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.Intake.RETRACT_SOLENOID);
  public final Solenoid extendSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.Intake.EXTEND_SOLENOID);
  /** Creates a new ExampleSubsystem. */
  public Intake() {
    setDefaultCommand(new DefaultIntake(this));
  }
    public void spin() {
        leftMotor.set(-speed);
        rightMotor.set(-speed);
    }

    public void setSpeed(double speed){
      leftMotor.set(-speed);
        rightMotor.set(-speed);
    }

    public void reverse() {
      leftMotor.set(speed);
      rightMotor.set(speed);
    }

    public double getMotorSpeed(){
      return -rightMotor.get();
    }

    public double getRPM(){
      return -rightEncoder.getVelocity();
    }

    public void stop() {
        leftMotor.set(0);
        rightMotor.set(0);
    }

    public boolean isStopped(){
      return leftMotor.get() == 0 && rightMotor.get() == 0;
    }

    public void extend() {
        retractSolenoid.set(false);
        extendSolenoid.set(true);
    }

    public void retract() {
      extendSolenoid.set(false);
      retractSolenoid.set(true);
    }

    public void rest(){
      extendSolenoid.set(false);
      retractSolenoid.set(false);
    }

    public boolean isRest(){
        return !extendSolenoid.get() && !retractSolenoid.get();
    }

    
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
