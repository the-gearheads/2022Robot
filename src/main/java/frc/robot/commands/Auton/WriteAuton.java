// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auton;

import java.util.ArrayList;
import java.util.HashMap;

import com.google.gson.Gson;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainInterface;

public class WriteAuton extends CommandBase {
  // private DriveTrainInterface driveTrain;
  private boolean started = false;
  private ArrayList<HashMap<String, Double>> recording = new ArrayList<HashMap<String, Double>>();
  private XboxController controller = new XboxController(Constants.Controller.PORT);
  private Joystick joy = new Joystick(Constants.Joystick.PORT);

  /** Creates a new CreateAuton. */
  public WriteAuton() {
    // this.driveTrain = driveTrain;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putBoolean("start recording", false);
    SmartDashboard.putBoolean("Running", true);
    started = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(SmartDashboard.getBoolean("start recording", false)){
      started = true;

      HashMap<String, Double> point = new HashMap<String, Double>();

      double lv = controller.getLeftY();
      double rh = controller.getRightX();
      double shoot = joy.getRawButton(6)?1:0;
      double intake = joy.getRawButton(1)?1:0;
      double elevator = joy.getRawButton(3)?1:0;

      point.put("lv", lv);
      point.put("rh",rh);
      point.put("shoot", shoot);
      point.put("intake", intake);
      point.put("elevator", elevator);


      recording.add(point);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    String json = (new Gson()).toJson(recording);
    SmartDashboard.putString("auton path", json);
    SmartDashboard.putBoolean("Running", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return started && !SmartDashboard.getBoolean("start recording", false);
  }
}
