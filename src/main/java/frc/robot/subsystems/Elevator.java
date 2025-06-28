package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.Intake.FillerDefaultElevate;

public class Elevator extends SubsystemBase{
  private SparkMax elevatorMotor = new SparkMax(Constants.Elevator.MOTOR_ID, MotorType.kBrushless);
  private final double speed = 0.75;

/** Creates a new ExampleSubsystem. */
  public Elevator(Intake intake, LightSensor lightSensor, ColorSensor colorSensor) {
    elevatorMotor.configure(new SparkMaxConfig().idleMode(IdleMode.kBrake), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // setDefaultCommand(new AutoElevate(lightSensor, colorSensor, this, intake, 1));
        setDefaultCommand(new FillerDefaultElevate(this));

  }

  @Override
  public void periodic() {
      // SmartDashboard.putNumber("Elevator Speed", elevatorMotor.get());
      if(Constants.Elevator.auto){
        SmartDashboard.putBoolean("Auto Elevate", true);
      }else{
        SmartDashboard.putBoolean("Auto Elevate", false);
      }
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
