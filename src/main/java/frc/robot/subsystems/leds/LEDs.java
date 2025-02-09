// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import frc.utils.FSMSubsystem;

public class LEDs extends FSMSubsystem {
    private final AddressableLED m_leds;
    private static final AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(133);
    private Alliance alliance = null;

    private int animationStep = 0;

    public LEDs() {
        setName("LEDs");
        m_leds = new AddressableLED(0);
        m_leds.setLength(m_ledBuffer.getLength());
        m_leds.setData(m_ledBuffer);
        m_leds.start();
    }

    @Override
    protected void enterNewState() {
        animationStep = 0;
    }

    @Override
    protected void exitCurrentState() {
        // Nothing to exit
    }

    @Override
    protected void executeCurrentStateBehavior() {
        LEDStates currentState = (LEDStates) getCurrentState();
        switch (currentState) {
            case IDLE:
                setIdlePattern();
                break;
            case INTAKED_CORAL:
                setCoralPattern();
                break;
            case INTAKED_ALGAE:
                setAlgaePattern();
                break;
            case INTAKED_BOTH:
                setBothPattern();
                break;
        }
        m_leds.setData(m_ledBuffer);
        animationStep = (animationStep + 1) % 180;
    }

    private void setIdlePattern() {
        Color allianceColor = (alliance == Alliance.Red) ? Color.kRed : Color.kBlue;
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setLED(i, allianceColor);
        }
    }

    private void setCoralPattern() {
        Color coral = Color.kOrangeRed;
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            if ((i + animationStep) % 5 == 0) {
                m_ledBuffer.setLED(i, coral);
            } else {
                m_ledBuffer.setLED(i, Color.kBlack);
            }
        }
    }

    private void setAlgaePattern() {
        Color algae = Color.kLime;
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            if ((i + animationStep) % 5 == 0) {
                m_ledBuffer.setLED(i, algae);
            } else {
                m_ledBuffer.setLED(i, Color.kBlack);
            }
        }
    }

    private void setBothPattern() {
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            final int hue = (animationStep + (i * 180 / m_ledBuffer.getLength())) % 180;
            m_ledBuffer.setHSV(i, hue, 255, 128);
        }
    }

    @Override
    public void stop() {
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setLED(i, Color.kBlack);
        }
        m_leds.setData(m_ledBuffer);
    }

    @Override
    public boolean isInState(Enum<?> state) {
        return getCurrentState() == state;
    }

    @Override
    public Enum<?> getCurrentState() {
        return currentState;
    }

    @Override
    protected Enum<?>[] getStates() {
        return LEDStates.values();
    }

    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
    }

    @Override
    public void periodic() {
        update();
        alliance = DriverStation.getAlliance().get();
    }

    public enum LEDStates {
        IDLE, INTAKED_CORAL, INTAKED_ALGAE, INTAKED_BOTH
    }
}

