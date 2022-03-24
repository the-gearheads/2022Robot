package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.Elevator.AutoElevate;

public class Elevator extends SubsystemBase{
  private CANSparkMax elevatorMotor = new CANSparkMax(Constants.Elevator.MOTOR_ID, MotorType.kBrushless);
  private final double speed = 1;

/** Creates a new ExampleSubsystem. */
  public Elevator(Intake intake, LightSensor lightSensor, ColorSensor colorSensor) {
    elevatorMotor.setIdleMode(IdleMode.kBrake);

    setDefaultCommand(new AutoElevate(lightSensor, colorSensor, this, intake, 0));
  }

  @Override
  public void periodic() {
      // SmartDashboard.putNumber("Elevator Speed", elevatorMotor.get());

    // This method will be called once per scheduler run
  }

  public void elevate(){
      elevatorMotor.set(speed);
  }

  public void reverse(){
      elevatorMotor.set(-speed);
  }

  public void stop(){
      elevatorMotor.set(0);
  }
}
