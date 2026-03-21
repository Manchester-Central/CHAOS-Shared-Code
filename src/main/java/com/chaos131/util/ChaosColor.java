package com.chaos131.util;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

// import edu.wpi.first.wpilibj.util.Color as WpiColor;

/** Colors stored in RGB format */
public enum ChaosColor {
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
  ChaosColor(int r, int g, int b) {
    this(new Color8Bit(r, g, b));
  }

  /** Forms a Color out of a Color8BIt */
  ChaosColor(Color8Bit c) {
    color8Bit = c;
    red = c.red;
    green = c.green;
    blue = c.blue;
  }

  /**
   * Returns a Color8Bit that shows scaled values for the duty cycle
   *
   * @param dutyCycle a percentage value [-1.0, 1.0] to convert to a scaled color
   */
  public static Color8Bit fromDutyCycle(double dutyCycle) {
    // If duty cycle is zero, show gray
    if (dutyCycle == 0) {
      return new Color8Bit(Color.kDimGray);
    }

    var h = dutyCycle < 0 ? 0 : 65; // red or green if negative or positive respectively
    var s = (int) Math.floor(Math.max(100.0, Math.abs(dutyCycle) * 255.0));

    return new Color8Bit(Color.fromHSV(h, s, 255));
  }
}
