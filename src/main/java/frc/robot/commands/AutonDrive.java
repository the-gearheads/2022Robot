// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class AutonDrive extends CommandBase {
  private DriveTrain driveTrain;
  private Trajectory trajectory;
  RamseteController ramsete = new RamseteController();
  private Timer timer = new Timer();
  private Pose2d initialPos;
  private List<Translation2d> interiorWaypoints;
  private Pose2d finalPos;

  /** Creates a new AutonDriveC. */
  public AutonDrive(DriveTrain driveTrain, Pose2d initialPos, List<Translation2d> interiorWaypoints, Pose2d finalPos) {
    this.driveTrain = driveTrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
    this.initialPos = initialPos;
    this.interiorWaypoints = interiorWaypoints;
    this.finalPos = finalPos;

  }

  public AutonDrive(DriveTrain driveTrain, List<Translation2d> interiorWaypoints, Pose2d finalPos) {
    this.driveTrain = driveTrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
    this.interiorWaypoints = interiorWaypoints;
    this.finalPos = finalPos;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    
    TrajectoryConfig config = new TrajectoryConfig(Constants.DriveTrain.MAX_VELOCITY * 2, Constants.DriveTrain.MAX_ACCELERATION * 1.5);
    config.setReversed(false);
    this.initialPos = driveTrain.getFieldPosition();
    trajectory = TrajectoryGenerator.generateTrajectory(
        initialPos,
        interiorWaypoints,
        finalPos,
        config);

        SmartDashboard.putString("Initial", "" + initialPos);
        SmartDashboard.putString("Final", "" + finalPos);
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrain.updateOdometry();
    Trajectory.State goal = trajectory.sample(timer.get());
    driveTrain.drive(ramsete.calculate(driveTrain.getFieldPosition(), goal));
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
