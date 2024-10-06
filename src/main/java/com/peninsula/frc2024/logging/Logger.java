package com.peninsula.frc2024.logging;

import com.peninsula.frc2024.logging.tree.LogTree;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructBuffer;
import edu.wpi.first.wpilibj.Timer;
import java.nio.ByteBuffer;
import java.util.HashMap;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.BlockingQueue;

@SuppressWarnings("unchecked")
public class Logger {
  LogTree currentData = new LogTree();

  LogWriter writer = new LogWriter();
  BlockingQueue<LogTree> dataBlockingQueue = new ArrayBlockingQueue<>(500);
  ReceiverQueue receiverQueue = new ReceiverQueue(dataBlockingQueue, writer);

  public HashMap<String, StructBuffer<?>> structBuffers = new HashMap<>();

  private static Logger sInstance;

  private boolean running = false;

  public static Logger getInstance() {
    if (sInstance == null) sInstance = new Logger();
    return sInstance;
  }

  public void start() {
    receiverQueue.start();
    running = true;
  }

  public void end() {
    receiverQueue.interrupt();
    running = false;
  }

  public void periodic() {
    if (running) {
      dataBlockingQueue.add(currentData.clone());
    }
  }

  public void log(String key, boolean value) {
    log(key, value, (long) (Timer.getFPGATimestamp() * 1_000_000));
  }

  public void log(String key, boolean value, long timestamp) {
    if (running) currentData.put(key, value, timestamp);
  }

  public void log(String key, long value) {
    log(key, value, (long) (Timer.getFPGATimestamp() * 1_000_000));
  }

  public void log(String key, long value, long timestamp) {
    if (running) currentData.put(key, value, timestamp);
  }

  public void log(String key, double value) {
    log(key, value, (long) (Timer.getFPGATimestamp() * 1_000_000));
  }

  public void log(String key, double value, long timestamp) {
    if (running) currentData.put(key, value, timestamp);
  }

  public void log(String key, int value) {
    log(key, value, (long) (Timer.getFPGATimestamp() * 1_000_000));
  }

  public void log(String key, int value, long timestamp) {
    if (running) currentData.put(key, value, timestamp);
  }

  public void log(String key, String value) {
    log(key, value, (long) (Timer.getFPGATimestamp() * 1_000_000));
  }

  public void log(String key, String value, long timestamp) {
    if (running) currentData.put(key, value, timestamp);
  }

  public void log(String key, boolean[] value) {
    log(key, value, (long) (Timer.getFPGATimestamp() * 1_000_000));
  }

  public void log(String key, boolean[] value, long timestamp) {
    if (running) currentData.put(key, value, timestamp);
  }

  public void log(String key, long[] value) {
    log(key, value, (long) (Timer.getFPGATimestamp() * 1_000_000));
  }

  public void log(String key, long[] value, long timestamp) {
    if (running) currentData.put(key, value, timestamp);
  }

  public void log(String key, double[] value) {
    log(key, value, (long) (Timer.getFPGATimestamp() * 1_000_000));
  }

  public void log(String key, double[] value, long timestamp) {
    if (running) currentData.put(key, value, timestamp);
  }

  public void log(String key, int[] value) {
    log(key, value, (long) (Timer.getFPGATimestamp() * 1_000_000));
  }

  public void log(String key, int[] value, long timestamp) {
    if (running) currentData.put(key, value, timestamp);
  }

  public void log(String key, String[] value) {
    log(key, value, (long) (Timer.getFPGATimestamp() * 1_000_000));
  }

  public void log(String key, String[] value, long timestamp) {
    if (running) currentData.put(key, value, timestamp);
  }

  public void log(String key, byte[] value) {
    log(key, value, (long) (Timer.getFPGATimestamp() * 1_000_000));
  }

  public void log(String key, byte[] value, long timestamp) {
    if (running) currentData.put(key, value, timestamp);
  }

  public void log(String key, Pose2d value) {
    log(key, value, (long) (Timer.getFPGATimestamp() * 1_000_000));
  }

  public void log(String key, Pose2d value, long timestamp) {
    if (running) log(key, Pose2d.struct, value, timestamp);
  }

  public void log(String key, Pose3d value) {
    log(key, value, (long) (Timer.getFPGATimestamp() * 1_000_000));
  }

  public void log(String key, Pose3d value, long timestamp) {
    if (running) log(key, Pose3d.struct, value, timestamp);
  }

  public <T> void log(String key, Struct<T> struct, T value) {
    if (running) log(key, struct, value, (long) (Timer.getFPGATimestamp() * 1_000_000));
  }

  public <T> void log(String key, Struct<T> struct, T value, long timestamp) {
    if (!structBuffers.containsKey(struct.getTypeString())) {
      writer.addSchema(struct);
      structBuffers.put(struct.getTypeString(), StructBuffer.create(struct));
    }
    StructBuffer<T> buffer = (StructBuffer<T>) structBuffers.get(struct.getTypeString());
    ByteBuffer bb = buffer.write(value);
    byte[] arr = new byte[bb.position()];
    bb.position(0);
    bb.get(arr);

    if (running) currentData.put(key, struct, arr, timestamp, false);
  }

  public <T> void log(String key, Struct<T> struct, T... value) {
    if (running) log(key, struct, (long) (Timer.getFPGATimestamp() * 1_000_000), value);
  }

  public <T> void log(String key, Struct<T> struct, long timestamp, T... value) {
    if (!structBuffers.containsKey(struct.getTypeString())) {
      writer.addSchema(struct);
      structBuffers.put(struct.getTypeString(), StructBuffer.create(struct));
    }
    StructBuffer<T> buffer = (StructBuffer<T>) structBuffers.get(struct.getTypeString());
    ByteBuffer bb = buffer.writeArray(value);
    byte[] arr = new byte[bb.position()];
    bb.position(0);
    bb.get(arr);

    if (running) currentData.put(key, struct, arr, timestamp, true);
  }
}
