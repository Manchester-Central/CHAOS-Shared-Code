package com.chaos131.robot;

import java.util.function.Supplier;

import com.chaos131.util.Color;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Class that manages addressable LEDs.
 * Does not do any patterns, sets all LEDs to the same color.
 */
public class LightStrip extends SubsystemBase {
    private AddressableLED m_leds;
    private AddressableLEDBuffer m_buffer;
    private final int m_portNumber;
    private final int m_numLEDs;
    private final Supplier<Color> m_colorPicker;

    public LightStrip(int port_number, int num_leds, Supplier<Color> colorPicker) {
        m_portNumber = port_number;
        m_numLEDs = num_leds;
        m_colorPicker = colorPicker;
        m_leds = new AddressableLED(m_portNumber);
        m_leds.setLength(m_numLEDs);
        m_buffer = new AddressableLEDBuffer(m_numLEDs);
        m_leds.start();
    }

    @Override
    public void periodic() {
        Color choosen_color = m_colorPicker.get();
        if (choosen_color != null) {
            setSingleColor(choosen_color);
        }
    }

    public void setSingleColor(int red, int green, int blue) {
        for(int i = 0; i < m_buffer.getLength(); i++) {
            m_buffer.setRGB(i, red, green, blue);
        }

        m_leds.setData(m_buffer);
    }

    public void setSingleColor(Color color) {
        setSingleColor(color.red, color.green, color.blue);
    }
}
