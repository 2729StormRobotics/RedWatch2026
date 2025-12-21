package frc.robot.util.misc;

// import static frc.robot.Constants.LookupTable; // TODO: Re-enable when 2026 game data is available

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Lookup table utility for interpolating values based on distance.
 * Currently disabled - re-enable and update when 2026 game data is available.
 */
public class Lookup {
  // TODO: Add 2026 game-specific lookup table constant
  // private static final double[][] LookupTable = { ... };
  // Remaps a value from the range l1 to r1 to a range l2 to r2
  // e.g. map(5,0,10,10,20) returns 15
  private static double map(double value, double l1, double r1, double l2, double r2) {
    if (r1 - l1 == 0) {
      return l1; // if these bounds are the same number, return that number (prevents divide by 0)
    }
    return (value - l1) * (r2 - l2) / (r1 - l1) + l2;
  }

  // returns the remapped value based on the which column in the lookup table (e.g. func(distance,
  // 1), would return remapped velocity)
  private static double getValueFromColumn(double distance, int column) {
    // TODO: Re-enable when 2026 game data is available
    // LookupTuner.updateMatrix(); // update the lookup table
    // int lowerIndex = 0; // the index of the value in the lookup table that is just greater than distance
    //
    // if (distance <= LookupTable[0][0]) {
    //   return map(distance, LookupTable[0][0], LookupTable[1][0], LookupTable[0][column], LookupTable[1][column]);
    // } else if (distance > LookupTable[LookupTable.length - 1][0]) {
    //   return map(distance, LookupTable[LookupTable.length - 2][0], LookupTable[LookupTable.length - 1][0],
    //       LookupTable[LookupTable.length - 2][column], LookupTable[LookupTable.length - 1][column]);
    // } else {
    //   while (distance > LookupTable[lowerIndex][0]) {
    //     lowerIndex++;
    //     if (lowerIndex == LookupTable.length) {
    //       lowerIndex -= 2;
    //       break;
    //     }
    //   }
    //   if (lowerIndex == LookupTable.length - 1) {
    //     lowerIndex = LookupTable.length - 2;
    //   }
    //   return map(distance, LookupTable[lowerIndex][0], LookupTable[lowerIndex + 1][0],
    //       LookupTable[lowerIndex][column], LookupTable[lowerIndex + 1][column]);
    // }
    return 0.0; // Placeholder - update when 2026 game data is available
  }

  /** Returns appropriate RPM based on distance. TODO: Re-enable when 2026 game data is available */
  public static double getRPM(double distance) {
    return getValueFromColumn(distance, 1); // 1 is the index of RPM
  }

  /** Returns the proper angle based on distance. TODO: Re-enable when 2026 game data is available */
  public static double getAngle(double distance) {
    return Rotation2d.fromDegrees(getValueFromColumn(distance, 2)).getRadians(); // 2 is the index of angle
  }
}
