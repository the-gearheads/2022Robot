// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LEDS;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDS;

public class SetShootingSeq extends CommandBase {
  private LEDS leds;
  private final int lenOfRunningSegments = 4;
  
  private int ledIndex = 0;
  private boolean seqDone = false;

  /** Creates a new setRedLEDS. */
  public SetShootingSeq(LEDS leds) {
    this.leds = leds;
    addRequirements(leds);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //leds.updateStrips(leds.purpleBuffer);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    leds.shootingBuffer.setRGB((leds.shootingBuffer.getLength() / 2) - ledIndex, 255, 0, 255);
    leds.shootingBuffer.setRGB((leds.shootingBuffer.getLength() / 2) + ledIndex, 255, 0, 255);
    leds.shootingBuffer.setRGB((leds.shootingBuffer.getLength() / 2) - ledIndex + lenOfRunningSegments, 0, 0, 0);
    leds.shootingBuffer.setRGB((leds.shootingBuffer.getLength() / 2) + ledIndex - lenOfRunningSegments, 0, 0, 0);
    leds.updateStrips(leds.shootingBuffer);

    ledIndex++;
    
    if(ledIndex == (leds.shootingBuffer.getLength() / 2)) {
      seqDone = true;
      leds.clearShootingBuffer();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    leds.clearShootingBuffer();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return seqDone;
  }
}
