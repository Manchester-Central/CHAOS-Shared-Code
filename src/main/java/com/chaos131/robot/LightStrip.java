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
    /** LED Controller */
    private AddressableLED m_leds;
    /** LED Buffer, to control each individual LED */
    private AddressableLEDBuffer m_buffer;
    /** Port on the RIO that connects to the LED Controller */
    private final int m_portNumber;
    /** Number of lights to manage */
    private final int m_numLEDs;
    /** Function to indicate which color should be choosen */
    private final Supplier<Color> m_colorPicker;

    /**
     * Constructs the LightStrip Controller
     * @param port_number on the rio
     * @param num_leds number of LEDs we will manage (must be less than or equal to the actual number of leds)
     * @param colorPicker supplier function to decide the color
     */
    public LightStrip(int port_number, int num_leds, Supplier<Color> colorPicker) {
        m_portNumber = port_number;
        m_numLEDs = num_leds;
        m_colorPicker = colorPicker;
        m_leds = new AddressableLED(m_portNumber);
        m_leds.setLength(m_numLEDs);
        m_buffer = new AddressableLEDBuffer(m_numLEDs);
        m_leds.start();
    }

    /**
     * Every periodic cycle, update the color with whatever the supplier indicates
     */
    @Override
    public void periodic() {
        Color choosen_color = m_colorPicker.get();
        if (choosen_color != null) {
            setSingleColor(choosen_color);
        }
    }

    /**
     * Sets every light to a single color
     * @param red amount
     * @param green amount
     * @param blue amount
     */
    public void setSingleColor(int red, int green, int blue) {
        for(int i = 0; i < m_buffer.getLength(); i++) {
            m_buffer.setRGB(i, red, green, blue);
        }

        m_leds.setData(m_buffer);
    }

    /**
     * Sets every light to a single color
     * @param color choosen
     */
    public void setSingleColor(Color color) {
        setSingleColor(color.red, color.green, color.blue);
    }
}
