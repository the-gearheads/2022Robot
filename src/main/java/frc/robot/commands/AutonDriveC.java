// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.spline.Spline.ControlVector;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainSS;

public class AutonDriveC extends CommandBase {
  private DriveTrainSS driveTrainSS;
  private Trajectory trajectory;
  RamseteController ramsete = new RamseteController();
  private double time = 0;

  /** Creates a new AutonDriveC. */
  public AutonDriveC(DriveTrainSS driveTrainSS, Pose2d initialPos, List<Translation2d> interiorWaypoints, Pose2d finalPos) {
    this.driveTrainSS = driveTrainSS;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrainSS);

    TrajectoryConfig config = new TrajectoryConfig(Constants.DriveTrain.MAX_VELOCITY, Constants.DriveTrain.MAX_ACCELERATION);
    config.setReversed(false);

    trajectory = TrajectoryGenerator.generateTrajectory(
        initialPos,
        interiorWaypoints,
        finalPos,
        config);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Trajectory.State goal = trajectory.sample(time);
    driveTrainSS.drive(ramsete.calculate(driveTrainSS.getFieldPosition(), goal));

    time+=20.0/1000;//20ms
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return  time >= trajectory.getTotalTimeSeconds();
  }
}
