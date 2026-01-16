// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.networktables.BooleanArrayPublisher;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.FloatArrayPublisher;
import edu.wpi.first.networktables.FloatPublisher;
import edu.wpi.first.networktables.IntegerArrayPublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.Publisher;
import edu.wpi.first.networktables.RawPublisher;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.WPISerializable;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.wpilibj.DriverStation;

import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.Map;

/**
 * A custom bare-bones logger for simple
 */
public final class Log {
  private static final NetworkTableInstance nt;

  private static final Map<String, Publisher> publishers = new HashMap<>();
  private static final Map<String, Struct<?>> structTypeCache = new HashMap<>();

  static {
    nt = NetworkTableInstance.getDefault();
  }

  /**
   * Don't use this
   */
  public Log() {
    throw new UnsupportedOperationException("You can't instantiate this class you bum");
  }

  public static void log(String identifier, int value) {
    ((IntegerPublisher)
            publishers.computeIfAbsent(identifier, k -> nt.getIntegerTopic(k).publish()))
        .set((Integer) value);
  }

  public static void log(String identifier, long value) {
    ((IntegerPublisher)
            publishers.computeIfAbsent(identifier, k -> nt.getIntegerTopic(k).publish()))
        .set(value);
  }

  public static void log(String identifier, float value) {
    ((FloatPublisher)
            publishers.computeIfAbsent(identifier, k -> nt.getFloatTopic(k).publish()))
        .set(value);
  }

  public static void log(String identifier, double value) {
    ((DoublePublisher)
            publishers.computeIfAbsent(identifier, k -> nt.getDoubleTopic(k).publish()))
        .set(value);
  }

  public static void log(String identifier, boolean value) {
    ((BooleanPublisher)
            publishers.computeIfAbsent(identifier, k -> nt.getBooleanTopic(k).publish()))
        .set(value);
  }

  public static void log(String identifier, byte[] value) {
    ((RawPublisher)
            publishers.computeIfAbsent(identifier, k -> nt.getRawTopic(k).publish("raw")))
        .set(value);
  }

  @SuppressWarnings("PMD.UnnecessaryCastRule")
  public static void log(String identifier, int[] value) {
    long[] widened = new long[value.length];

    for (int i = 0; i < value.length; i++) {
      widened[i] = (long) value[i];
    }

    ((IntegerArrayPublisher)
            publishers.computeIfAbsent(identifier, k -> nt.getIntegerArrayTopic(k).publish()))
        .set(widened);
  }

  public static void log(String identifier, long[] value) {
    ((IntegerArrayPublisher)
            publishers.computeIfAbsent(identifier, k -> nt.getIntegerArrayTopic(k).publish()))
        .set(value);
  }

  public static void log(String identifier, float[] value) {
    ((FloatArrayPublisher)
            publishers.computeIfAbsent(identifier, k -> nt.getFloatArrayTopic(k).publish()))
        .set(value);
  }

  public static void log(String identifier, double[] value) {
    ((DoubleArrayPublisher)
            publishers.computeIfAbsent(identifier, k -> nt.getDoubleArrayTopic(k).publish()))
        .set(value);
  }

  public static void log(String identifier, boolean[] value) {
    ((BooleanArrayPublisher)
            publishers.computeIfAbsent(identifier, k -> nt.getBooleanArrayTopic(k).publish()))
        .set(value);
  }

  public static void log(String identifier, String value) {
    ((StringPublisher)
            publishers.computeIfAbsent(identifier, k -> nt.getStringTopic(k).publish()))
        .set(value);
  }

  public static void log(String identifier, String[] value) {
    ((StringArrayPublisher)
            publishers.computeIfAbsent(identifier, k -> nt.getStringArrayTopic(k).publish()))
        .set(value);
  }
  
  @SuppressWarnings("unchecked")
  public static <T> void log(String identifier, Struct<T> struct, T value) {
    nt.addSchema(struct);
    ((StructPublisher<T>)
            publishers.computeIfAbsent(identifier, k -> nt.getStructTopic(k, struct).publish()))
        .set(value);
  }
  
  @SuppressWarnings("unchecked")
  public static <T> void log(String identifier, Struct<T> struct, T[] value) {
    nt.addSchema(struct);
    ((StructArrayPublisher<T>)
            publishers.computeIfAbsent(
                identifier, k -> nt.getStructArrayTopic(k, struct).publish()))
        .set(value);
  }

  @SuppressWarnings("unchecked")
  public static <T extends WPISerializable> void log(String identifier, T value) {
    if (value == null)
      return;
    Struct<T> struct = (Struct<T>) findStructType(value.getClass());
    if (struct != null) {
      log(identifier, struct, value);
    } else {
      DriverStation.reportError(
          "Log.java: Auto serialization is not supported for type " + value.getClass().getSimpleName(),
          false);
    }
  }

  private static Struct<?> findStructType(Class<?> classObj) {
    if (!structTypeCache.containsKey(classObj.getName())) {
      structTypeCache.put(classObj.getName(), null);
      Field field = null;
      try {
        field = classObj.getDeclaredField("struct");
      } catch (NoSuchFieldException | SecurityException e) {
      }
      if (field != null) {
        try {
          structTypeCache.put(classObj.getName(), (Struct<?>) field.get(null));
        } catch (IllegalArgumentException | IllegalAccessException e) {
        }
      }
    }
    return structTypeCache.get(classObj.getName());
  }
}
