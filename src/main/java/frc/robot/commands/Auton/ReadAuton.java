// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auton;

import java.io.Reader;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.HashMap;

import com.google.gson.Gson;
import com.google.gson.internal.LinkedTreeMap;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;
import frc.robot.Constants;
import frc.robot.commands.Shooter.ActuateShooter;

public class ReadAuton extends CommandBase {
  Reader reader;
  Gson gson = new Gson();
  private Shooter shooter;
  private Intake intake;
  private DriveTrainInterface driveTrain;
  private ArrayList<LinkedTreeMap<String, Double>> recording = new ArrayList<LinkedTreeMap<String, Double>>();
  private int pointer;
  private Elevator elevator;
  private String autonPath;

  /** Creates a new ReadAuton. */
  public ReadAuton(DriveTrainInterface driveTrain, Intake intake, Elevator elevator, Shooter shooter, String auton) {
    this.driveTrain = driveTrain;
    this.intake = intake;
    this.shooter = shooter;
    this.elevator = elevator;
    this.autonPath = auton;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public ReadAuton(DriveTrainInterface driveTrain, String auton) {
    this.driveTrain = driveTrain;
    this.autonPath = auton;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    try{
      this.reader = Files.newBufferedReader(Paths.get("/home/lvuser/deploy/autons/" + autonPath));
    }catch(Exception e){
      SmartDashboard.putString("ERROR", "Error in ReadAuton.java: " + e.getMessage() + e.getLocalizedMessage());
      e.printStackTrace();
    }
    pointer = 0;
    this.recording = gson.fromJson(reader, recording.getClass());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    LinkedTreeMap prevPoint = null;
    if (pointer > 0) {
      prevPoint = recording.get(pointer - 1);
    }

    LinkedTreeMap point = recording.get(pointer);

    double lv = (double)point.get("lv");
    double rh = (double)point.get("rh");
    double shoot = (double) point.get("shoot");
    double intaking = (double) point.get("intake");
    double elevating = (double) point.get("elevator");
    boolean flag = false;

    if ((shoot > 0.5 && (pointer == 0 || ((double) prevPoint.get("shoot")) < 0.5))){
      (new ActuateShooter(shooter, 0.15, 0.15, true, true)).schedule();
    }

    if (intaking > 0.5 && (pointer == 0 || (double) prevPoint.get("intake") < 0.5)) {
      flag = true;
      intake.extend();
      intake.spin();
      elevator.elevate();
    } else if (intaking < 0.5 && (pointer == 0 || ((double) prevPoint.get("intake")) > 0.5)) {
      flag = false;
      intake.retract();
      intake.stop();
      elevator.stop();
    }

    if (elevating> 0.5 && (pointer == 0 || (double) prevPoint.get("elevator") < 0.5)) {
      elevator.elevate();
    } else if (elevating < 0.5 && (pointer == 0 || ((double) prevPoint.get("elevator")) > 0.5) && !flag) {
      elevator.stop();
    }

    SmartDashboard.putBoolean("intake", ((double) point.get("intake")) > 0.5);
    SmartDashboard.putBoolean("shoot", ((double) point.get("shoot")) > 0.5);
    SmartDashboard.putNumber("lv", (double)point.get("lv"));
    SmartDashboard.putNumber("rh", (double)point.get("rh"));

    driveTrain.drive(lv * Constants.DriveTrain.MAX_VELOCITY, rh * Constants.DriveTrain.MAX_ROT_VELOCITY);

    pointer++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.drive(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pointer >= recording.size();
  }
}