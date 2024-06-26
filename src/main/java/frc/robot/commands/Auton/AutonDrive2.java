// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auton;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriveTrainInterface;

public class AutonDrive2 extends Command {
  private DriveTrainInterface driveTrain;
  private Trajectory trajectory;
  RamseteController ramsete = new RamseteController();
  private Timer timer = new Timer();
  private Pose2d initialPos;
  private List<Translation2d> interiorWaypoints;
  private Pose2d finalPos;
  private boolean isBackward = false;
  private double requestedXInches;

  /** Creates a new AutonDriveC. */
  public AutonDrive2(DriveTrainInterface driveTrain) {
    this.driveTrain = driveTrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
    this.initialPos = new Pose2d(0,0,new Rotation2d(0));
    this.interiorWaypoints = new ArrayList<>();
  }

  public AutonDrive2(DriveTrainInterface driveTrain, double requestedXInches) {
    this.driveTrain = driveTrain;
    this.requestedXInches = requestedXInches;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
    this.initialPos = new Pose2d(0,0,new Rotation2d(0));
    this.interiorWaypoints = new ArrayList<>();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.setFieldPos(new Pose2d(0,0, new Rotation2d(0)));
    double xDirectionality = requestedXInches / Math.abs(requestedXInches);
    double processedXInches = xDirectionality * Math.pow(Math.abs((requestedXInches)/0.135), (1.0/2.04));
    double processedXMeters = processedXInches * 0.0254;

    // double requestedYInches = SmartDashboard.getNumber("Y val", 0);
    // // double processedXInches = (requestedYInches + 5.6) / 0.89;
    // double processedYMeters = requestedYInches * 0.0254;

    // double requestedAngle = SmartDashboard.getNumber("Angle", 0);
    
    // this.finalPos = new Pose2d(processedXMeters,processedYMeters, new Rotation2d(requestedAngle));
    
    TrajectoryConfig config = new TrajectoryConfig(2.2, 0.3);//*2 and *1.5
    config.setReversed(requestedXInches < 0);

    trajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0,0, new Rotation2d(0)),
        interiorWaypoints,
        new Pose2d(processedXMeters,0, new Rotation2d(0)),
        config);
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrain.updateOdometry();
    Trajectory.State goal = trajectory.sample(timer.get());
    ChassisSpeeds ramseteValue = ramsete.calculate(driveTrain.getFieldPosition(), goal);
    driveTrain.drive(ramseteValue);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return  timer.get() >= trajectory.getTotalTimeSeconds();
  }
}
