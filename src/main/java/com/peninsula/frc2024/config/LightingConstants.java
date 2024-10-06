package com.peninsula.frc2024.config;

import com.ctre.phoenix.led.*;

public class LightingConstants {
  public static double minVoltageToFunction = 12.2;

  /** Rainbow * */
  public static final Animation rainbow = new RainbowAnimation(0.5, 0.8, 43);

  public static final Animation greenBlink = new StrobeAnimation(0, 255, 0, 0, 0.5, 43);
  public static final Animation blueBlink = new StrobeAnimation(0, 0, 255, 0, 0.5, 43);
  public static final Animation goldBlink = new StrobeAnimation(179, 99, 19, 0, 0.5, 43);

  public static final Animation orangeSolid = new StrobeAnimation(190, 20, 0, 0, 0.0, 43);
  public static final Animation redSolid = new StrobeAnimation(255, 0, 0, 0, 0.0, 43);
  public static final Animation goldSolid = new StrobeAnimation(179, 99, 19, 0, 0.0, 43);

  public static final Animation lowBattery =
      new ColorFlowAnimation(255, 0, 0, 0, 0.5, 43, ColorFlowAnimation.Direction.Forward);
}
