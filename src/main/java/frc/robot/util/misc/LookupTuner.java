package frc.robot.util.misc;

// import static frc.robot.Constants.LookupTable; // TODO: Re-enable when 2026 game data is available

/**
 * Lookup table tuner utility for adjusting lookup table values via NetworkTables.
 * Currently disabled - re-enable and update when 2026 game data is available.
 */
public class LookupTuner {
  // TODO: Re-enable when 2026 game data is available
  // private static LoggedTunableNumber[][] lookupTable =
  //     new LoggedTunableNumber[LookupTable.length][LookupTable[0].length];

  /** Sets up the tuner. TODO: Re-enable when 2026 game data is available */
  public static void setupTuner() {
    // TODO: Re-enable when 2026 game data is available
    // for (int i = 0; i < LookupTable.length; i++) {
    //   for (int j = 1; j < LookupTable[i].length; j++) {
    //     System.out.print(LookupTable[i][j] + " ");
    //     String text = j == 1 ? "RPM" : "Angle";
    //     lookupTable[i][j] =
    //         new LoggedTunableNumber(
    //             "LookupTable/" + "Meters-" + LookupTable[i][0] + "/" + text, LookupTable[i][j]);
    //   }
    // }
  }

  /** Gets a matrix value. TODO: Re-enable when 2026 game data is available */
  public static double getMatrixValue(int i, int j) {
    // TODO: Re-enable when 2026 game data is available
    // return lookupTable[i][j].get();
    return 0.0; // Placeholder
  }

  /** Updates the matrix. TODO: Re-enable when 2026 game data is available */
  public static void updateMatrix() {
    // TODO: Re-enable when 2026 game data is available
    // for (int i = 0; i < LookupTable.length; i++) {
    //   for (int j = 1; j < LookupTable[i].length; j++) {
    //     if (lookupTable[i][j].get() != LookupTable[i][j]) {
    //       LookupTable[i][j] = lookupTable[i][j].get();
    //     }
    //   }
    // }
  }
}
