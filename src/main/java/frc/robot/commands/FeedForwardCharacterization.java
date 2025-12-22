// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.LinkedList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class FeedForwardCharacterization extends Command {
  private static final double START_DELAY_SECS = 2.0;
  private static final double RAMP_VOLTS_PER_SEC = 0.1;

  private FeedForwardCharacterizationData data;
  private final Consumer<Double> voltageConsumer;
  private final Supplier<Double> velocitySupplier;

  private final Timer timer = new Timer();

  /** Creates a new FeedForwardCharacterization command. */
  public FeedForwardCharacterization(
      Subsystem subsystem, Consumer<Double> voltageConsumer, Supplier<Double> velocitySupplier) {
    addRequirements(subsystem);
    this.voltageConsumer = voltageConsumer;
    this.velocitySupplier = velocitySupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    data = new FeedForwardCharacterizationData();
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.get() < START_DELAY_SECS) {
      voltageConsumer.accept(0.0);
    } else {
      double voltage = (timer.get() - START_DELAY_SECS) * RAMP_VOLTS_PER_SEC;
      voltageConsumer.accept(voltage);
      data.add(velocitySupplier.get(), voltage);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    voltageConsumer.accept(0.0);
    timer.stop();
    data.print();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public static class FeedForwardCharacterizationData {
    private final List<Double> velocityData = new LinkedList<>();
    private final List<Double> voltageData = new LinkedList<>();

    public void add(double velocity, double voltage) {
      if (Math.abs(velocity) > 1E-4) {
        velocityData.add(Math.abs(velocity));
        voltageData.add(Math.abs(voltage));
      }
    }

    public void print() {
      if (velocityData.size() == 0 || voltageData.size() == 0) {
        return;
      }

      RegressionResults regression = calculateLinearRegression();
      if (regression == null) {
        System.out.println("FF Characterization Results: Unable to compute regression (degenerate data)");
        SmartDashboard.putString("FF Characterization/Status", "Regression failed");
        return;
      }

      System.out.println("FF Characterization Results:");
      System.out.println("\tCount=" + Integer.toString(velocityData.size()) + "");
      System.out.println(String.format("\tR2=%.5f", regression.r2));
      System.out.println(String.format("\tkS=%.5f", regression.kS));
      System.out.println(String.format("\tkV=%.5f", regression.kV));

      SmartDashboard.putNumber("FF Characterization/Count", velocityData.size());
      SmartDashboard.putNumber("FF Characterization/R2", regression.r2);
      SmartDashboard.putNumber("FF Characterization/kS", regression.kS);
      SmartDashboard.putNumber("FF Characterization/kV", regression.kV);
      SmartDashboard.putString("FF Characterization/Status", "OK");
    }

    private RegressionResults calculateLinearRegression() {
      int n = velocityData.size();
      if (n < 2) {
        return null;
      }

      double sumX = 0.0;
      double sumY = 0.0;
      double sumXY = 0.0;
      double sumXX = 0.0;
      for (int i = 0; i < n; i++) {
        double x = velocityData.get(i);
        double y = voltageData.get(i);
        sumX += x;
        sumY += y;
        sumXY += x * y;
        sumXX += x * x;
      }

      double denominator = (n * sumXX) - (sumX * sumX);
      if (Math.abs(denominator) < 1E-9) {
        return null;
      }

      double kV = ((n * sumXY) - (sumX * sumY)) / denominator;
      double kS = (sumY - (kV * sumX)) / n;

      double meanY = sumY / n;
      double ssTot = 0.0;
      double ssRes = 0.0;
      for (int i = 0; i < n; i++) {
        double x = velocityData.get(i);
        double y = voltageData.get(i);
        double predicted = kS + (kV * x);
        ssTot += Math.pow(y - meanY, 2);
        ssRes += Math.pow(y - predicted, 2);
      }

      double r2 = ssTot > 1E-9 ? 1.0 - (ssRes / ssTot) : 0.0;
      return new RegressionResults(kS, kV, r2);
    }

    private static class RegressionResults {
      final double kS;
      final double kV;
      final double r2;

      RegressionResults(double kS, double kV, double r2) {
        this.kS = kS;
        this.kV = kV;
        this.r2 = r2;
      }
    }
  }
}
