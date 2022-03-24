// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DistanceSensor extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  AnalogInput analog = new AnalogInput(2);
  private final double lower = 6.7; //closest distance to hub
  private final double upper = 36; //farthest distance to hub
  private NetworkTableEntry display;
  public DistanceSensor() {
    display = Shuffleboard.getTab("SmartDashboard").add("Distance", false).withWidget(BuiltInWidgets.kBooleanBox).getEntry();
  }

  public void displayInRange(boolean inRange){
    if(inRange){
      display.setBoolean(true);
    }else{
      display.setBoolean(false);
    }
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

  private boolean isInRange(double lower, double upper){
    return !(getDistanceInInches() < lower || getDistanceInInches() > upper);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Distance In CM", getDistanceInCM());
    // SmartDashboard.putNumber("Distance In Inches", getDistanceInInches());
    displayInRange(isInRange(lower, upper));
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
