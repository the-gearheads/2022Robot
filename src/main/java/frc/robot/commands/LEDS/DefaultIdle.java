// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LEDS;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.LEDS;

public class DefaultIdle extends Command {
  /** Creates a new DefaultIdle. */
  private LEDS leds;

  private int direction = 1;
  private int ledIndex = 0;

  public DefaultIdle(LEDS leds) {
    this.leds = leds;
    addRequirements(leds);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ledIndex = 0;
    leds.clearBuffer(leds.liveBuffer);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

      leds.liveBuffer.setRGB(
        ledIndex, 
        Constants.LEDS.red,  // team colors
        Constants.LEDS.green,
        Constants.LEDS.blue
      );

      if (ledIndex == leds.liveBuffer.getLength() - 1) {
        direction = -1;
      }
      else if (ledIndex == 0) {
        direction = 1;        
      }

      ledIndex += 1 * direction;
      leds.updateStrips(leds.liveBuffer);
      leds.clearBufferYellow(leds.liveBuffer);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
