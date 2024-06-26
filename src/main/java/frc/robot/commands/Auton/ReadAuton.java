// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auton;

import java.io.Reader;
import java.lang.reflect.Type;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;

import com.google.gson.Gson;
import com.google.gson.reflect.TypeToken;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.*;


public class ReadAuton extends Command {
  Reader reader;
  Gson gson = new Gson();
  private DriveTrainInterface driveTrain;
  private ArrayList<Pose2d> recording = new ArrayList<Pose2d>();
  private String autonPath;
  RamseteController ramsete;
  private boolean isBackward;
  private Trajectory trajectory;
  private Timer timer;
  private boolean isPlay;


  /** Creates a new ReadAuton. */
  public ReadAuton(DriveTrainInterface driveTrain, String auton, boolean isBackward) {
    this.driveTrain = driveTrain;
    this.autonPath = auton;
    this.isBackward = isBackward;
    this.isPlay = false;

    // Get Trajectory
    try{
      this.reader = Files.newBufferedReader(Paths.get("/home/lvuser/deploy/autons/" + autonPath + ".json"));
    }catch(Exception e){
      SmartDashboard.putString("ERROR", "Error in ReadAuton.java: " + e.getMessage() + e.getLocalizedMessage());
      e.printStackTrace();
    }
    Type castType = new TypeToken<ArrayList<Pose2d>>(){}.getType(); 
    this.recording = gson.fromJson(reader, castType);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public ReadAuton(DriveTrainInterface driveTrain, boolean isBackward) {
    this.driveTrain = driveTrain;
    this.isBackward = isBackward;
    this.isPlay = true;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(isPlay){
      String pathJSON = SmartDashboard.getString("auton path", "");
      Type castType = new TypeToken<ArrayList<Pose2d>>(){}.getType(); 
      this.recording = gson.fromJson(pathJSON, castType);
      SmartDashboard.putString("Here is what I see", gson.toJson(recording));
    }
    //Set field pos to 0
    driveTrain.setFieldPos(new Pose2d(0,0, new Rotation2d(0)));
      
    //Set up trajectory
    ramsete = new RamseteController();
    TrajectoryConfig config = new TrajectoryConfig(0.6, 0.2);//*2 and *1.5
    config.setReversed(isBackward);
    SmartDashboard.putString("Recording", recording.get(0).toString());
    trajectory = TrajectoryGenerator.generateTrajectory(recording, config);

    //Start Timer
    timer = new Timer();
    timer.reset();
    timer.start();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Run ramsete controller ig
    driveTrain.updateOdometry();
    Trajectory.State goal = trajectory.sample(timer.get());
    ChassisSpeeds ramseteValue = ramsete.calculate(driveTrain.getFieldPosition(), goal);
    if(timer.get() > (2.0/3) * trajectory.getTotalTimeSeconds()){
      ramseteValue.omegaRadiansPerSecond /= 2;
    }
    driveTrain.drive(ramseteValue);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //Don't want that bot to keep moving!
    driveTrain.drive(0,0);
    // if(recording.get(recording.size() - 1).getTranslation().getDistance(driveTrain.getFieldPosition().getTranslation()) > 0.01){
    //   (new AutonDrive(driveTrain, driveTrain.getFieldPosition(), recording.get(recording.size() - 1))).schedule();
    // }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //Please stop at some point
    // return true;
    return timer.get() > trajectory.getTotalTimeSeconds();
  }
}


// public class ReadAuton extends Command {
//   Reader reader;
//   Gson gson = new Gson();
//   private Shooter shooter;
//   private Intake intake;
//   private DriveTrainInterface driveTrain;
//   private ArrayList<LinkedTreeMap<String, Double>> recording = new ArrayList<LinkedTreeMap<String, Double>>();
//   private int pointer;
//   private Elevator elevator;
//   private String autonPath;

//   /** Creates a new ReadAuton. */
//   public ReadAuton(DriveTrainInterface driveTrain, Intake intake, Elevator elevator, Shooter shooter, String auton) {
//     this.driveTrain = driveTrain;
//     this.intake = intake;
//     this.shooter = shooter;
//     this.elevator = elevator;
//     this.autonPath = auton;
//     // Use addRequirements() here to declare subsystem dependencies.
//   }

//   public ReadAuton(DriveTrainInterface driveTrain, String auton) {
//     this.driveTrain = driveTrain;
//     this.autonPath = auton;
//     // Use addRequirements() here to declare subsystem dependencies.
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     try{
//       this.reader = Files.newBufferedReader(Paths.get("/home/lvuser/deploy/autons/" + autonPath));
//     }catch(Exception e){
//       SmartDashboard.putString("ERROR", "Error in ReadAuton.java: " + e.getMessage() + e.getLocalizedMessage());
//       e.printStackTrace();
//     }
//     pointer = 0;
//     this.recording = gson.fromJson(reader, recording.getClass());
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     LinkedTreeMap prevPoint = null;
//     if (pointer > 0) {
//       prevPoint = recording.get(pointer - 1);
//     }

//     LinkedTreeMap point = recording.get(pointer);

//     double lv = (double)point.get("lv");
//     double rh = (double)point.get("rh");
//     double shoot = (double) point.get("shoot");
//     double intaking = (double) point.get("intake");
//     double elevating = (double) point.get("elevator");
//     boolean flag = false;

//     if ((shoot > 0.5 && (pointer == 0 || ((double) prevPoint.get("shoot")) < 0.5))){
//       (new ActuateShooter(shooter, 0.15, 0.15, true, true)).schedule();
//     }

//     if (intaking > 0.5 && (pointer == 0 || (double) prevPoint.get("intake") < 0.5)) {
//       flag = true;
//       intake.extend();
//       intake.spin();
//       elevator.elevate();
//     } else if (intaking < 0.5 && (pointer == 0 || ((double) prevPoint.get("intake")) > 0.5)) {
//       flag = false;
//       intake.retract();
//       intake.stop();
//       elevator.stop();
//     }

//     if (elevating> 0.5 && (pointer == 0 || (double) prevPoint.get("elevator") < 0.5)) {
//       elevator.elevate();
//     } else if (elevating < 0.5 && (pointer == 0 || ((double) prevPoint.get("elevator")) > 0.5) && !flag) {
//       elevator.stop();
//     }

//     SmartDashboard.putBoolean("intake", ((double) point.get("intake")) > 0.5);
//     SmartDashboard.putBoolean("shoot", ((double) point.get("shoot")) > 0.5);
//     SmartDashboard.putNumber("lv", (double)point.get("lv"));
//     SmartDashboard.putNumber("rh", (double)point.get("rh"));

//     driveTrain.drive(lv * Constants.DriveTrain.MAX_VELOCITY, rh * Constants.DriveTrain.MAX_ROT_VELOCITY);

//     pointer++;
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     driveTrain.drive(0,0);
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return pointer >= recording.size();
//   }
// }
