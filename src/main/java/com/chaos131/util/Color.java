package com.chaos131.util;

import edu.wpi.first.wpilibj.util.Color8Bit;

/** Colors stored in RGB format */
public enum Color {
  /** Red */
  RED(255, 0, 0),
  /** Green */
  GREEN(0, 255, 0),
  /** Blue */
  BLUE(0, 0, 255),
  /** Team's trademark orange color */
  CHAOS_ORANGE(255, 30, 0),
  /** Off */
  OFF(0, 0, 0),
  /** Yellow */
  YELLOW(255, 255, 0),
  /** Pure White */
  WHITE(255, 255, 255),
  /** Purple */
  PURPLE(255, 0, 255);

  public Color8Bit color8Bit;

  /** Color's red channel */
  public int red;

  /** Color's green channel */
  public int green;

  /** Color's blue channel */
  public int blue;

  /** Forms a Color out of numbers */
  Color(int r, int g, int b) {
    this(new Color8Bit(r, g, b));
  }

  /** Forms a Color out of a Color8BIt */
  Color(Color8Bit c) {
    color8Bit = c;
    red = c.red;
    green = c.green;
    blue = c.blue;
  }

  /**
   * Returns a COlor8Bit that shows scaled values for the duty cycle
   *
   * @param dutyCycle a percentage value [-1.0, 1.0] to convert to a scaled color
   */
  public static Color8Bit fromDutyCycle(double dutyCycle) {
    int grayBase = 131;
    // If duty cycle is zero, show gray
    if (dutyCycle == 0) {
      return new Color8Bit(grayBase, grayBase, grayBase);
    }

    int scaleValue = (int) Math.floor(100 * dutyCycle);
    int scaledUpValue = grayBase + scaleValue + 10;
    int scaledDownValue = grayBase - scaleValue - 10;
    return dutyCycle > 0
        ? new Color8Bit(
            scaledDownValue,
            scaledUpValue,
            scaledDownValue) // Show a scaled green value if positive
        : new Color8Bit(
            scaledUpValue, scaledDownValue, scaledDownValue); // Show a scaled red value if negatve
  }
}
