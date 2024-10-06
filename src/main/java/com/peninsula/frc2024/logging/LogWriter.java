package com.peninsula.frc2024.logging;

import com.peninsula.frc2024.logging.tree.LogTree;
import com.peninsula.frc2024.robot.Robot;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.struct.Struct;
import java.util.HashMap;
import java.util.Map;

/** called by receiver thread when it gets something in the queue * */
public class LogWriter {
  Map<String, Integer> entryId = new HashMap<>();

  private final double writePeriodSecs = 0.25;
  private String name = "test", folder = "/media/sda1/logs/";

  public DataLog log;

  public void start() {

    if (!Robot.isRobotReal()) {
      folder = System.getProperty("user.dir") + "/logs";
    }

    log = new DataLog(folder, name + ".wpilog", writePeriodSecs);
  }

  public void addSchema(Struct<?> struct) {
    log.addSchema(struct);
  }

  public void putData(LogTree data) {
    for (Map.Entry<String, LogTree.LogValue> a : data.getData().entrySet()) {
      if (!entryId.containsKey(a.getKey())) {
        if (a.getValue().datatype == LogTree.LogValue.type.STRUCT) {
          entryId.put(a.getKey(), log.start(a.getKey(), a.getValue().structType, "", 0));
        } else {
          entryId.put(a.getKey(), log.start(a.getKey(), a.getValue().datatype.wpi_type, "", 0));
        }
      }
      append(a.getKey(), a.getValue());
    }
  }

  public void append(String key, LogTree.LogValue value) {
    switch (value.datatype) {
      case INT -> log.appendInteger(entryId.get(key), (int) value.val, value.timestamp);
      case LONG -> log.appendInteger(entryId.get(key), (long) value.val, value.timestamp);
      case BOOLEAN -> log.appendBoolean(entryId.get(key), (boolean) value.val, value.timestamp);
      case DOUBLE -> log.appendDouble(entryId.get(key), (double) value.val, value.timestamp);
      case STRING -> log.appendString(entryId.get(key), (String) value.val, value.timestamp);

      case BOOLEAN_A -> log.appendBooleanArray(
          entryId.get(key), (boolean[]) value.val, value.timestamp);
      case DOUBLE_A -> log.appendDoubleArray(
          entryId.get(key), (double[]) value.val, value.timestamp);
      case INT_A, LONG_A -> log.appendIntegerArray(
          entryId.get(key), (long[]) value.val, value.timestamp);
      case STRING_A -> log.appendStringArray(
          entryId.get(key), (String[]) value.val, value.timestamp);
      case BYTE_A, STRUCT -> log.appendRaw(entryId.get(key), (byte[]) value.val, value.timestamp);

      default -> throw new IllegalArgumentException("Unsupported data type: " + value.datatype);
    }
  }

  public void end() {
    log.close();
  }
}
