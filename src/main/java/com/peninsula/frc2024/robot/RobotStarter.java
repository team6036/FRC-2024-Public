package com.peninsula.frc2024.robot;

import edu.wpi.first.wpilibj.RobotBase;

public final class RobotStarter {
  //
  //  static {
  //    // Disables warnings about Kryonet. See https://github.com/EsotericSoftware/kryo/issues/692
  //    try {
  //      Class<?> loggerClass = Class.forName("jdk.internal.module.IllegalAccessLogger");
  //      Field loggerField = loggerClass.getDeclaredField("logger");
  //      Class<?> unsafeClass = Class.forName("sun.misc.Unsafe");
  //      Field unsafeField = unsafeClass.getDeclaredField("theUnsafe");
  //      unsafeField.setAccessible(true);
  //      Object unsafe = unsafeField.get(null);
  //      var offset =
  //          (Long)
  //              unsafeClass.getMethod("staticFieldOffset", Field.class).invoke(unsafe,
  // loggerField);
  //      unsafeClass
  //          .getMethod("putObjectVolatile", Object.class, long.class, Object.class)
  //          .invoke(unsafe, loggerClass, offset, null);
  //    } catch (Exception exception) {
  //      Log.warn(
  //          String.format("Failed to disable illegal access warning for Kryonet: %s", exception));
  //    }
  //  }

  private RobotStarter() {}

  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }
}
