// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.LEDS.DefaultRainbow;

public class LEDS extends SubsystemBase {
  /** Creates a new LEDS. */
  private AddressableLED ledStrip;
  
  // one buffer for all the live shit, one for no lights
  public AddressableLEDBuffer liveBuffer;
  public AddressableLEDBuffer nullBuffer;
  public AddressableLEDBuffer purpleBuffer;
  public AddressableLEDBuffer greenBuffer;
  public AddressableLEDBuffer shootingBuffer;

  // Assuming that both strips are the same length - therefore we can use one buffer
  private int length = 0;
  public int test = 0;

  public LEDS(int port , int stripLength) {
    ledStrip = new AddressableLED(port);
    length = stripLength;
    
    // initialize buffers
    liveBuffer = new AddressableLEDBuffer(length);
    nullBuffer = new AddressableLEDBuffer(length);
    purpleBuffer = new AddressableLEDBuffer(length);
    greenBuffer = new AddressableLEDBuffer(length);
    shootingBuffer = new AddressableLEDBuffer(length);

    fillNullBuffer();
    fillRainbowBuffer();
    fillPurpleBuffer();
    fillGreenBuffer();

    // both strips same length
    ledStrip.setLength(length);

    // set both strips to null
    updateStrips(nullBuffer);

    // send voltages to the strips
    startStrips();

    setDefaultCommand(new DefaultRainbow(this));
  }

  // update both strips to a new buffer
  public void updateStrips(AddressableLEDBuffer buffer) {
    ledStrip.setData(buffer);
  }

  // method to flush both strips
  public void startStrips() {
    ledStrip.start();
  }
  
  // fill null buffer with (0, 0, 0)
  public void fillNullBuffer() {
    for (int i = 0; i < nullBuffer.getLength(); i++) {
      nullBuffer.setRGB(i, 0, 0, 0);
    }
  }

  public void fillRainbowBuffer() {
    for (int i = 0; i < liveBuffer.getLength(); i++) {
      liveBuffer.setHSV(i, (test + (i * 180 / liveBuffer.getLength())) % 180, 255, 255);
    }
  }

  public void fillPurpleBuffer() {
    for (int i = 0; i < purpleBuffer.getLength(); i++) {
      purpleBuffer.setRGB(i, 255, 0, 255);
    }
  }

  public void fillGreenBuffer() {
    for (int i = 0; i < greenBuffer.getLength(); i++) {
      greenBuffer.setRGB(i, 0, 255, 0);
    }
  }

  public void clearShootingBuffer() {
    for (int i = 0; i < shootingBuffer.getLength(); i++) {
      shootingBuffer.setRGB(i, 0, 0, 0);
    }
  }

  @Override
  public void periodic() {
  }
}