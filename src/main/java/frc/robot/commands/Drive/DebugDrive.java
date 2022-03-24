// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.DriveTrainInterface;

public class DebugDrive extends CommandBase {
  private DriveTrainInterface driveTrain;
  private final XboxController controller = new XboxController(Constants.Controller.PORT);

  /** Creates a new DebugDrive. */
  public DebugDrive(DriveTrainInterface driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("left constant", Constants.DriveTrain.LEFT_SPEED_CONSTANT); 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Constants.DriveTrain.LEFT_SPEED_CONSTANT = SmartDashboard.getNumber("left constant", Constants.DriveTrain.LEFT_SPEED_CONSTANT);
    double lvAxis = Constants.DriveTrain.FORWARD_DIRECTION  * controller.getLeftY();
    double rhAxis = -controller.getRightX();

    lvAxis = MathUtil.applyDeadband(lvAxis, 0.2);
    rhAxis = MathUtil.applyDeadband(rhAxis, 0.2);

    double xSpeed = lvAxis * Constants.DriveTrain.MAX_VELOCITY;
    double rotSpeed = rhAxis * Constants.DriveTrain.MAX_ROT_VELOCITY;

    driveTrain.debugDrive(xSpeed, rotSpeed);
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
