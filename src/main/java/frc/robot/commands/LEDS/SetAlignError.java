// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LEDS;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDS;

public class SetAlignError extends CommandBase {
  /** Creates a new setGreenLEDS. */
  private LEDS leds;

  private boolean seqDone = false;
  private Timer flashingTimer = new Timer();

  public SetAlignError(LEDS leds) {
    this.leds = leds;
    addRequirements(leds);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    leds.updateStrips(leds.orangeBuffer);
    leds.startStrips();

    flashingTimer.reset();
    flashingTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (flashingTimer.hasElapsed(0.25))
    {
      leds.updateStrips(leds.nullBuffer);
    }
    if (flashingTimer.hasElapsed(0.5))
    {
      leds.updateStrips(leds.orangeBuffer);
    }
    if (flashingTimer.hasElapsed(0.75))
    {
      leds.updateStrips(leds.nullBuffer);
    }
    if (flashingTimer.hasElapsed(1))
    {
      seqDone = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    flashingTimer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return seqDone;
  }
}
