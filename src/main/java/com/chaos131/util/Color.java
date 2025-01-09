package com.chaos131.util;

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

  /** Color's red channel */
  public int red;

  /** Color's green channel */
  public int green;

  /** Color's blue channel */
  public int blue;

  /** Forms a color out of numbers */
  Color(int r, int g, int b) {
    red = r;
    green = g;
    blue = b;
  }
}
