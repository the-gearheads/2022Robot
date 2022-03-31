package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * This is a simple example to show how the REV Color Sensor V3 can be used to
 * detect pre-configured colors.
 */
public class LightSensor extends SubsystemBase {
  DigitalInput lightSensor = new DigitalInput(Constants.LightSensor.PORT);
  public LightSensor(){
  }
  public boolean get(){
    return !lightSensor.get();
  }
  @Override
  public void periodic() {
    // SmartDashboard.putBoolean("Light Sensor", get());
  }
}
