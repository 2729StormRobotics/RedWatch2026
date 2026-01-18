// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.function.DoubleSupplier;

/**
 * Provides an interface for asynchronously reading high-frequency measurements to a set of queues.
 *
 * <p>This version includes an overload for Spark signals, which checks for errors to ensure that
 * all measurements in the sample are valid.
 */
public class SparkOdometryThread {
  private final List<SparkBase> sparks = new ArrayList<>();
  private final List<DoubleSupplier> sparkSignals = new ArrayList<>();
  private final List<DoubleSupplier> genericSignals = new ArrayList<>();
  private final List<Queue<Double>> sparkQueues = new ArrayList<>();
  private final List<Queue<Double>> genericQueues = new ArrayList<>();
  private final List<Queue<Double>> timestampQueues = new ArrayList<>();

  private static SparkOdometryThread instance = null;
  private Notifier notifier = new Notifier(this::run);

  public static SparkOdometryThread getInstance() {
    if (instance == null) {
      instance = new SparkOdometryThread();
    }
    return instance;
  }

  private SparkOdometryThread() {
    notifier.setName("OdometryThread");
  }

  public void start() {
    if (timestampQueues.size() > 0) {
      notifier.startPeriodic(1.0 / DriveConstants.odometryFrequency);
    }
  }

  /** Registers a Spark signal to be read from the thread. */
  public Queue<Double> registerSignal(SparkBase spark, DoubleSupplier signal) {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    Drive.odometryLock.lock();
    try {
      sparks.add(spark);
      sparkSignals.add(signal);
      sparkQueues.add(queue);
    } finally {
      Drive.odometryLock.unlock();
    }
    return queue;
  }

  /** Registers a generic signal to be read from the thread. */
  public Queue<Double> registerSignal(DoubleSupplier signal) {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    Drive.odometryLock.lock();
    try {
      genericSignals.add(signal);
      genericQueues.add(queue);
    } finally {
      Drive.odometryLock.unlock();
    }
    return queue;
  }

  /** Returns a new queue that returns timestamp values for each sample. */
  public Queue<Double> makeTimestampQueue() {
    Queue<Double> queue = new ArrayBlockingQueue<>(20);
    Drive.odometryLock.lock();
    try {
      timestampQueues.add(queue);
    } finally {
      Drive.odometryLock.unlock();
    }
    return queue;
  }

  private void run() {
  Drive.odometryLock.lock();
  try {
    double timestamp = RobotController.getFPGATime() / 1e6;

    // Read Spark values, track validity separately
    double[] sparkValues = new double[sparkSignals.size()];
    boolean sparkValid = true;
    for (int i = 0; i < sparkSignals.size(); i++) {
      sparkValues[i] = sparkSignals.get(i).getAsDouble();
      if (sparks.get(i).getLastError() != REVLibError.kOk) {
        sparkValid = false;
      }
    }

    // Read generic signals (gyro) ALWAYS - don't tie to Spark validity
    double[] genericValues = new double[genericSignals.size()];
    for (int i = 0; i < genericSignals.size(); i++) {
      genericValues[i] = genericSignals.get(i).getAsDouble();
    }

    // Add Spark data only if valid
    if (sparkValid) {
      for (int i = 0; i < sparkSignals.size(); i++) {
        sparkQueues.get(i).offer(sparkValues[i]);
      }
      // Add generic signals and timestamps together with Spark data
      for (int i = 0; i < genericSignals.size(); i++) {
        genericQueues.get(i).offer(genericValues[i]);
      }
      for (Queue<Double> queue : timestampQueues) {
        queue.offer(timestamp);
      }
    }
  } finally {
    Drive.odometryLock.unlock();
  }
}
}