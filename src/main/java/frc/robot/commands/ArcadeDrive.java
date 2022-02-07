// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriveTrain2;

public class ArcadeDrive extends CommandBase {
  private DriveTrain driveTrain;

  /** Creates a new ArcadeDriveC. */
  public ArcadeDrive(DriveTrain driveTrain) {
    this.driveTrain = driveTrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

// Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double lvAxis = -DriverStation.getStickAxis(Constants.Controller.PORT, Constants.Controller.LV_AXIS);
    double rhAxis = -DriverStation.getStickAxis(Constants.Controller.PORT, Constants.Controller.RH_AXIS);

    // driveTrain.drive(lvAxis * C, rhAxis);
    driveTrain.drive(lvAxis * Constants.DriveTrain.MAX_VELOCITY, rhAxis * Constants.DriveTrain.MAX_ROT_VELOCITY);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.drive(0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
