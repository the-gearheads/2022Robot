// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LEDS;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDS;

public class DefaultRainbow extends Command {
  private LEDS leds;

  /** Creates a new defaultLED. */
  public DefaultRainbow(LEDS leds) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.leds = leds;
    addRequirements(leds);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    leds.test = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    leds.fillRainbowBuffer();
    leds.updateStrips(leds.liveBuffer);
    leds.test += 5;
    leds.test %= 180;
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
