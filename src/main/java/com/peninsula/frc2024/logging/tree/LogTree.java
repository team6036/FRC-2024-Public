package com.peninsula.frc2024.logging.tree;

import edu.wpi.first.util.struct.Struct;
import java.util.HashMap;
import java.util.Map;

public class LogTree {

  Map<String, LogValue> data;

  public LogTree() {
    data = new HashMap<>();
  }

  public LogTree(Map<String, LogValue> data) {
    this.data = data;
  }

  public Map<String, LogValue> getData() {
    return data;
  }

  @Override
  public LogTree clone() {
    Map<String, LogValue> data2 = new HashMap<>(data);
    return new LogTree(data2);
  }

  public void put(String key, boolean value, long timestamp) {
    data.put(key, new LogValue(value, timestamp));
  }

  public void put(String key, long value, long timestamp) {
    data.put(key, new LogValue(value, timestamp));
  }

  public void put(String key, double value, long timestamp) {
    data.put(key, new LogValue(value, timestamp));
  }

  public void put(String key, int value, long timestamp) {
    data.put(key, new LogValue(value, timestamp));
  }

  public void put(String key, String value, long timestamp) {
    data.put(key, new LogValue(value, timestamp));
  }

  public void put(String key, boolean[] value, long timestamp) {
    data.put(key, new LogValue(value, timestamp));
  }

  public void put(String key, long[] value, long timestamp) {
    data.put(key, new LogValue(value, timestamp));
  }

  public void put(String key, double[] value, long timestamp) {
    data.put(key, new LogValue(value, timestamp));
  }

  public void put(String key, int[] value, long timestamp) {
    data.put(key, new LogValue(value, timestamp));
  }

  public void put(String key, String[] value, long timestamp) {
    data.put(key, new LogValue(value, timestamp));
  }

  public void put(String key, byte[] value, long timestamp) {
    data.put(key, new LogValue(value, timestamp));
  }

  public void put(String key, Struct<?> struct, byte[] value, long timestamp, boolean arr) {
    data.put(key, new LogValue(value, struct, timestamp, arr));
  }

  public class LogValue {
    public enum type {
      BOOLEAN("boolean"),
      LONG("long"),
      DOUBLE("double"),
      INT("int64"),
      STRING("string"),
      BOOLEAN_A("boolean[]"),
      LONG_A("long[]"),
      DOUBLE_A("double[]"),
      INT_A("int[]"),
      STRING_A("string[]"),
      BYTE_A("byte[]"),
      STRUCT;

      public String wpi_type;

      type(String s) {
        wpi_type = s;
      }

      type() {}
    }

    public type datatype;
    public Object val;
    public long timestamp = 0; // microseconds
    public String structType;

    public LogValue(byte[] value, Struct<?> struct, long timestamp, boolean arr) {
      this.val = value;
      datatype = type.STRUCT;
      structType = struct.getTypeString();
      if (arr) structType += "[]";
      this.timestamp = timestamp;
    }

    public LogValue(boolean value, long timestamp) {
      this.val = value;
      datatype = type.BOOLEAN;
      this.timestamp = timestamp;
    }

    public LogValue(long value, long timestamp) {
      this.val = value;
      datatype = type.LONG;
      this.timestamp = timestamp;
    }

    public LogValue(double value, long timestamp) {
      this.val = value;
      datatype = type.DOUBLE;
      this.timestamp = timestamp;
    }

    public LogValue(int value, long timestamp) {
      this.val = value;
      datatype = type.INT;
      this.timestamp = timestamp;
    }

    public LogValue(String value, long timestamp) {
      this.val = value;
      datatype = type.STRING;
      this.timestamp = timestamp;
    }

    public LogValue(boolean[] value, long timestamp) {
      this.val = value;
      datatype = type.BOOLEAN_A;
      this.timestamp = timestamp;
    }

    public LogValue(long[] value, long timestamp) {
      this.val = value;
      datatype = type.LONG_A;
      this.timestamp = timestamp;
    }

    public LogValue(double[] value, long timestamp) {
      this.val = value;
      datatype = type.DOUBLE_A;
      this.timestamp = timestamp;
    }

    public LogValue(int[] value, long timestamp) {
      this.val = value;
      datatype = type.INT_A;
      this.timestamp = timestamp;
    }

    public LogValue(String[] value, long timestamp) {
      this.val = value;
      datatype = type.STRING_A;
      this.timestamp = timestamp;
    }

    public LogValue(byte[] value, long timestamp) {
      this.val = value;
      datatype = type.BYTE_A;
      this.timestamp = timestamp;
    }
  }
}
