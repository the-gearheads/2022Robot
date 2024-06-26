package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;

import java.util.Map;

import com.revrobotics.ColorMatch;
import frc.robot.Constants;

/**
 * This is a simple example to show how the REV Color Sensor V3 can be used to
 * detect pre-configured colors.
 */
public class ColorSensor extends SubsystemBase {
  /**
   * A Rev Color Sensor V3 object is constructed with an I2C port as a 
   * parameter. The device will be automatically initialized with default 
   * parameters.
   */
  private final ColorSensorV3 colorSensor = new ColorSensorV3(Constants.ColorSensor.PORT);

  /**
   * A Rev Color Match object is used to register and detect known colors. This can 
   * be calibrated ahead of time or during operation.
   * 
   * This object uses a simple euclidian distance to estimate the closest match
   * with given confidence range.
   */
  private final ColorMatch colorMatcher = new ColorMatch();
  private final GenericHID controller = new GenericHID(1);

  /**
   * Note: Any example colors should be calibrated as the user needs, these
   * are here as a basic example.
   */
  private final Color kBlueTarget = new Color(0.185, 0.426, 0.39);
  private final Color kRedTarget = new Color(0.44, 0.39, 0.17);
  private SimpleWidget display;

  public ColorSensor(){
    colorMatcher.addColorMatch(kBlueTarget);
    colorMatcher.addColorMatch(kRedTarget);
    display = Shuffleboard.getTab("SmartDashboard").add("Color", false).withWidget(BuiltInWidgets.kBooleanBox);
  }

  public boolean displayColor(String color){
      if(color.equals("red")){
          display.withProperties(Map.of("Color when true", "#E74C3C", "Color when false", "#E74C3C")).getEntry();  
      }else if(color.equals("blue")){
        display.withProperties(Map.of("Color when true", "#85C1E9", "Color when false", "#85C1E9")).getEntry();  
    }else if(color.equals("unknown")){
        display.withProperties(Map.of("Color when true", "#4d4d4d", "Color when false", "#4d4d4d")).getEntry();  
    }else{
        return false;
    }
    return true;
  }
  @Override
  public void periodic() {
      double[] rgb = getRGB();
      // SmartDashboard.putString("Color", "Red: " + rgb[0] + "; Green: " + rgb[1] + "; Blue: " + rgb[2]);
    //   SmartDashboard.putNumber("Proximity", getProximity());
    displayColor(redOrBlue());
      
  }

  public double[] getRGB(){
    Color detectedColor = colorSensor.getColor();
    return new double[] {detectedColor.red, detectedColor.green, detectedColor.blue};
  }

  public double getProximity(){
    return colorSensor.getProximity();
  }

  public String redOrBlue(){
/**
     * The method GetColor() returns a normalized color value from the sensor and can be
     * useful if outputting the color to an RGB LED or similar. To
     * read the raw color, use GetRawColor().
     * 
     * The color sensor works best when within a few inches from an object in
     * well lit conditions (the built in LED is a big help here!). The farther
     * an object is the more light from the surroundings will bleed into the 
     * measurements and make it difficult to accurately determine its color.
     */
    Color detectedColor = colorSensor.getColor();

    /**
     * Run the color match algorithm on our detected color
     */
    String colorString = "";
    ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);

    if(getProximity()<120) {
        colorString = "unknown";
    }else if (match.color == kBlueTarget) {
        colorString = "blue";
    } else if (match.color == kRedTarget) {
        colorString = "red";
    }

    return colorString;
  }
}
