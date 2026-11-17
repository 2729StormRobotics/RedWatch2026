//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * Contains generic field dimensions that are common across FRC seasons.
 * All units are in meters.
 * 
 * Note: Game-specific field elements should be added here when the 2026 game is announced.
 */
public class FieldConstants {
  /** Field length in meters */
  public static final double fieldLength = Units.inchesToMeters(690.876);

  /** Field width in meters */
  public static final double fieldWidth = Units.inchesToMeters(317);

  /** Starting line X position in meters (measured from the inside of starting line) */
  public static final double startingLineX = Units.inchesToMeters(299.438);
}