// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DistanceSensor extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  AnalogInput analog = new AnalogInput(1);
  public DistanceSensor() {
    SmartDashboard.putNumber("kP", 0.03);
  }

  public double getDistanceInCM(){
    double voltage_scale_factor = 5/RobotController.getVoltage5V();
    double distanceCentimeters = analog.getValue() * voltage_scale_factor * 0.125;
    double distanceWithinRobot = 10 * 2.54;
    return round(distanceCentimeters - distanceWithinRobot, 1);
  }
  public double getDistanceInInches(){
    double voltage_scale_factor = 5/RobotController.getVoltage5V();
    double distanceInches = analog.getValue() * voltage_scale_factor * 0.0492;
    double distanceWithinRobot = 10;
    return round(distanceInches - distanceWithinRobot, 1);
  }

  private double round(double value, double decimalPlaces){
    return ((int)(value * Math.pow(10, decimalPlaces))) / Math.pow(10,decimalPlaces);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Distance In CM", getDistanceInCM());
    SmartDashboard.putNumber("Distance In Inches", getDistanceInInches());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
