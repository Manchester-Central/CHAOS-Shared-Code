package com.chaos131.util;

public enum Color {
    RED (255, 0, 0),
    GREEN (0, 255, 0),
    BLUE (0, 0, 255),
    CHAOS_ORANGE (255, 30, 0),
    OFF (0, 0, 0),
    YELLOW (255, 255, 0),
    WHITE (255, 255, 255),
    PURPLE (255, 0, 255);
    
    public int red;
    public int green;
    public int blue;

    Color(int r, int g, int b) {
        red = r;
        green = g;
        blue = b;
    }
}