package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;

/**
 * A Singleton controller for the REV Robotics Blinkin LED Driver.
 * Optimized for a single LED strip across the entire robot.
 */
public class BlinkinLEDController extends SubsystemBase {

    /**
     * All possible Blinkin patterns as defined in the REV manual.
     */
    public enum BlinkinPattern {
        // Fixed Palette Patterns
        RAINBOW_RAINBOW_PALETTE(-0.99), RAINBOW_PARTY_PALETTE(-0.97), RAINBOW_OCEAN_PALETTE(-0.95),
        RAINBOW_LAVA_PALETTE(-0.93), RAINBOW_FOREST_PALETTE(-0.91), RAINBOW_WITH_GLITTER(-0.89),
        CONFETTI(-0.87), SHOT_RED(-0.85), SHOT_BLUE(-0.83), SHOT_WHITE(-0.81),
        SINELON_RAINBOW_PALETTE(-0.79), SINELON_PARTY_PALETTE(-0.77), SINELON_OCEAN_PALETTE(-0.75),
        SINELON_LAVA_PALETTE(-0.73), SINELON_FOREST_PALETTE(-0.71), BPM_RAINBOW_PALETTE(-0.69),
        BPM_PARTY_PALETTE(-0.67), BPM_OCEAN_PALETTE(-0.65), BPM_LAVA_PALETTE(-0.63),
        BPM_FOREST_PALETTE(-0.61), FIRE_MEDIUM(-0.59), FIRE_LARGE(-0.57),
        TWINKLES_RAINBOW_PALETTE(-0.55), TWINKLES_PARTY_PALETTE(-0.53), TWINKLES_OCEAN_PALETTE(-0.51),
        TWINKLES_LAVA_PALETTE(-0.49), TWINKLES_FOREST_PALETTE(-0.47), COLOR_WAVES_RAINBOW_PALETTE(-0.45),
        COLOR_WAVES_PARTY_PALETTE(-0.43), COLOR_WAVES_OCEAN_PALETTE(-0.41), COLOR_WAVES_LAVA_PALETTE(-0.39),
        COLOR_WAVES_FOREST_PALETTE(-0.37), LARSON_SCANNER_RED(-0.35), LARSON_SCANNER_GRAY(-0.33),
        LIGHT_CHASE_RED(-0.31), LIGHT_CHASE_BLUE(-0.29), LIGHT_CHASE_GRAY(-0.27),
        HEARTBEAT_RED(-0.25), HEARTBEAT_BLUE(-0.23), HEARTBEAT_WHITE(-0.21), HEARTBEAT_GRAY(-0.19),
        BREATH_RED(-0.17), BREATH_BLUE(-0.15), BREATH_GRAY(-0.13), STROBE_RED(-0.11),
        STROBE_BLUE(-0.09), STROBE_GOLD(-0.07), STROBE_WHITE(-0.05),

        // Color 1 Patterns
        CP1_END_TO_END_BLEND_TO_BLACK(-0.03), CP1_LARSON_SCANNER(-0.01), CP1_LIGHT_CHASE(0.01),
        CP1_HEARTBEAT_SLOW(0.03), CP1_HEARTBEAT_MEDIUM(0.05), CP1_HEARTBEAT_FAST(0.07),
        CP1_BREATH_SLOW(0.09), CP1_BREATH_FAST(0.11), CP1_SHOT(0.13), CP1_STROBE(0.15),

        // Color 2 Patterns
        CP2_END_TO_END_BLEND_TO_BLACK(0.17), CP2_LARSON_SCANNER(0.19), CP2_LIGHT_CHASE(0.21),
        CP2_HEARTBEAT_SLOW(0.23), CP2_HEARTBEAT_MEDIUM(0.25), CP2_HEARTBEAT_FAST(0.27),
        CP2_BREATH_SLOW(0.29), CP2_BREATH_FAST(0.31), CP2_SHOT(0.33), CP2_STROBE(0.35),

        // Color 1 & 2 Patterns
        CP1_2_SPARKLE_1_ON_2(0.37), CP1_2_SPARKLE_2_ON_1(0.39), CP1_2_COLOR_GRADIENT(0.41),
        CP1_2_BPM(0.43), CP1_2_END_TO_END_BLEND_1_TO_2(0.45), CP1_2_END_TO_END_BLEND(0.47),
        CP1_2_NO_BLENDING(0.49), CP1_2_TWINKLES(0.51), CP1_2_COLOR_WAVES(0.53), CP1_2_SINELON(0.55),

        // Solid Colors
        HOT_PINK(0.57), DARK_RED(0.59), RED(0.61), RED_ORANGE(0.63), ORANGE(0.65), GOLD(0.67),
        YELLOW(0.69), LAWN_GREEN(0.71), LIME(0.73), DARK_GREEN(0.75), GREEN(0.77),
        BLUE_GREEN(0.79), AQUA(0.81), SKY_BLUE(0.83), DARK_BLUE(0.85), BLUE(0.87),
        BLUE_VIOLET(0.89), VIOLET(0.91), WHITE(0.93), GRAY(0.95), DARK_GRAY(0.97), BLACK(0.99);

        public final double value;
        BlinkinPattern(double value) { this.value = value; }
    }

    private static BlinkinLEDController m_instance;
    private final Spark m_blinkin;
    private BlinkinPattern m_currentPattern = BlinkinPattern.BLACK;

    // Robot State Flags
    public boolean isEnabled = false;
    public boolean isEndgame = false;
    public boolean tagsSeen = false;
    public boolean readyToFire = false;
    public boolean passing = false;
    public boolean meltdown = false;
    public boolean stall = false;

    /**
     * Get the singleton instance of the BlinkinLEDController.
     * Uses a default PWM port if not already initialized.
     */
    public static BlinkinLEDController getInstance() {
        if (m_instance == null) {
            // Default port 0, can be changed here or in a custom init
            m_instance = new BlinkinLEDController(0); 
        }
        return m_instance;
    }

    /**
     * Private constructor for singleton pattern.
     * @param pwmPort The PWM port on the RoboRIO.
     */
    private BlinkinLEDController(int pwmPort) {
        m_blinkin = new Spark(pwmPort);
    }

    /**
     * Directly set the LED pattern.
     * @param pattern The pattern to display.
     */
    public void setPattern(BlinkinPattern pattern) {
        if (pattern == null) return;
        m_currentPattern = pattern;
        m_blinkin.set(m_currentPattern.value);
    }

    /**
     * Sets the LEDs based on the current Alliance color.
     * @param solid If true, sets solid color. If false, sets a heartbeat/breath.
     */
    public void setAllianceColor(boolean solid) {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isEmpty()) {
            setPattern(BlinkinPattern.GRAY);
            return;
        }

        if (alliance.get() == Alliance.Red) {
            setPattern(solid ? BlinkinPattern.RED : BlinkinPattern.HEARTBEAT_RED);
        } else {
            setPattern(solid ? BlinkinPattern.BLUE : BlinkinPattern.HEARTBEAT_BLUE);
        }
    }

    @Override
    public void periodic() {
        // Priority-based LED logic
        if (meltdown) {
            setPattern(BlinkinPattern.FIRE_LARGE);
        } else if (stall) {
            setPattern(BlinkinPattern.STROBE_WHITE);
        } else if (readyToFire && tagsSeen) {
            // Solid Green when ready to fire
            setPattern(BlinkinPattern.GREEN);
        } else if (tagsSeen) {
            // Blinking/Strobe Green when vision tags are seen
            // Note: Configure Color 1 on the Blinkin hardware to be Green
            setPattern(BlinkinPattern.CP1_STROBE); 
        } else if (passing) {
            setPattern(BlinkinPattern.VIOLET);
        } else if (isEndgame) {
            setPattern(BlinkinPattern.RAINBOW_PARTY_PALETTE);
        } else if (isEnabled) {
            setAllianceColor(true);
        } else {
            // Default idle pattern
            setPattern(BlinkinPattern.CP1_2_SPARKLE_1_ON_2);
        }

        // Telemetry
        Logger.recordOutput("LEDs/Pattern", m_currentPattern.toString());
        Logger.recordOutput("LEDs/PWMValue", m_currentPattern.value);
    }

    /** Turn off the LEDs. */
    public void off() { setPattern(BlinkinPattern.BLACK); }

    /** Get the currently active pattern. */
    public BlinkinPattern getCurrentPattern() { return m_currentPattern; }
}