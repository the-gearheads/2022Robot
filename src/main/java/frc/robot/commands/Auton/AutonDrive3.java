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
import frc.robot.subsystems.DriveTrainInterface;

public class AutonDrive3 extends Command {
  private DriveTrainInterface driveTrain;
  private Trajectory trajectory;
  RamseteController ramsete = new RamseteController();
  private Timer timer = new Timer();
  private List<Translation2d> interiorWaypoints;
  private boolean isBackward = false;
  private double requestedX;
  private double requestedY;
  private double requestedAngle;

  public AutonDrive3(DriveTrainInterface driveTrain, double requestedX, double requestedY, double requestedAngle, boolean isBackward) {
    this.driveTrain = driveTrain;
    this.requestedX = requestedX;
    this.requestedY = requestedY;
    this.requestedAngle = requestedAngle;
    this.isBackward = isBackward;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
    this.interiorWaypoints = new ArrayList<>();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.setFieldPos(driveTrain.getFieldPosition());
    Pose2d initialPos = driveTrain.getFieldPosition();
    Pose2d finalPos = new Pose2d(requestedX, requestedY, new Rotation2d(requestedAngle));
    
    double middleX = (initialPos.getX() + finalPos.getX()) / 2;
    double middleY = (initialPos.getY() + finalPos.getY()) / 2;

    double firstX = (initialPos.getX() + middleX) / 2;
    double firstY = (initialPos.getY() + middleY) / 2;

    double lastX = (finalPos.getX() + middleX) / 2;
    double lastY = (finalPos.getY() + middleY) / 2;

    Translation2d firstPoint = new Translation2d(firstX, firstY);
    Translation2d middlePoint = new Translation2d(middleX, middleY);
    Translation2d lastPoint = new Translation2d(lastX, lastY);

    interiorWaypoints.add(firstPoint);
    interiorWaypoints.add(middlePoint);
    interiorWaypoints.add(lastPoint);
    TrajectoryConfig config = new TrajectoryConfig(SmartDashboard.getNumber("Velocity",0), SmartDashboard.getNumber("Acc",0));//*2 and *1.5
    config.setReversed(isBackward);

    trajectory = TrajectoryGenerator.generateTrajectory(
      initialPos,
      interiorWaypoints,
      finalPos,
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
    ramseteValue.vxMetersPerSecond*=-1;
    ramseteValue.omegaRadiansPerSecond*=1/1.5;
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
