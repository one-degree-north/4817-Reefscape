// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotState;

public class LEDs extends SubsystemBase {
    private static final int LED_PORT = 0; // Adjust this to match your LED port
    private static final int LED_COUNT = 133; // Adjust this to match your LED strip

    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_ledBuffer;

    public boolean zeroed = false;

    private LEDPattern currentPattern;

    public LEDs() {
        m_led = new AddressableLED(LED_PORT);
        m_ledBuffer = new AddressableLEDBuffer(LED_COUNT);
        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);
        m_led.start();
        setDefaultCommand(runPattern(LEDPattern.solid(Color.kBlack)).withName("Off"));
    }

    public Command runPattern(LEDPattern pattern) {
        return run(() -> pattern.applyTo(m_ledBuffer));
    }

    private void setLEDPattern() {
        if (RobotState.isEnabled()) {
            if (zeroed) {
                currentPattern = LEDPattern.solid(Color.kGreen);
            } else {
                currentPattern = LEDPattern.solid(Color.kRed);
            }
        } else {
            currentPattern = LEDPattern.rainbow(200, 20);
        }
    }

    @Override
    public void periodic() {
        setLEDPattern(); // Automatically set the pattern based on robot state
        currentPattern.applyTo(m_ledBuffer);
        m_led.setData(m_ledBuffer);
    }
}

