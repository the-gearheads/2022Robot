// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auton;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.DriveTrainInterface;

public class PreparedAuton extends CommandBase {
  private DriveTrainInterface driveTrain;
  private Trajectory trajectory;
  private String trajectoryJSON;
  private Timer timer;
  private RamseteController ramsete;

  /** Creates a new PreparedAuton. */
  public PreparedAuton(DriveTrainInterface driveTrain, String trajectoryJSON) {
    this.driveTrain = driveTrain;
    this.trajectoryJSON = trajectoryJSON;
    this.timer = new Timer();
    this.ramsete = new RamseteController();
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.trajectory = new Trajectory();
    try {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(this.trajectoryJSON);
        this.trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + this.trajectoryJSON, ex.getStackTrace());
    }
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrain.updateOdometry();
    Trajectory.State goal = trajectory.sample(timer.get());
    ChassisSpeeds ramseteValue = ramsete.calculate(driveTrain.getFieldPosition(), goal);
    ramseteValue.vxMetersPerSecond *=-1;
    ramseteValue.omegaRadiansPerSecond/=1.5;
    driveTrain.drive(ramseteValue);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    driveTrain.drive(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return trajectory.getTotalTimeSeconds() < timer.get();
  }
}
