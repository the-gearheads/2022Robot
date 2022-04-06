// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auton;

import java.util.ArrayList;
import java.util.HashMap;

import com.google.gson.Gson;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainInterface;

public class WriteAuton extends CommandBase {
  // private DriveTrainInterface driveTrain;
  // private boolean started = false;
  private ArrayList<Pose2d> recording;
  private DriveTrainInterface driveTrain;
  private int discountedNum = 0;

  /** Creates a new CreateAuton. */
  public WriteAuton(DriveTrainInterface driveTrain) {
    this.driveTrain = driveTrain;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    recording = new ArrayList<Pose2d>();
    //Put in smartdashboard booleans to start recording
    // SmartDashboard.putBoolean("start recording", false);
    SmartDashboard.putBoolean("Running", true);
    // started = false;

    //Set position to 0
    driveTrain.setFieldPos(new Pose2d(0,0, new Rotation2d(0)));
    recording.add(new Pose2d(0,0, new Rotation2d(0)));

    discountedNum = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // start recording
    // if(SmartDashboard.getBoolean("start recording", false)){
    //   started = true;
      Pose2d point = driveTrain.getFieldPosition();
      if(point.getTranslation().getDistance(recording.get(recording.size() - 1).getTranslation()) > 1E-12){
        recording.add(point);
      }else{
        SmartDashboard.putNumber("discountedNum", ++discountedNum);
      }
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //Paste recording so I can copy and paste that into a json file
    String json = (new Gson()).toJson(recording);
    SmartDashboard.putString("auton path", json);
    SmartDashboard.putBoolean("Running", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Please finish at some point
    // return started && !SmartDashboard.getBoolean("start recording", false);
    return false;
  }
}
