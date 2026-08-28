package org.firstinspires.ftc.teamcode.core.util;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.BooleanSupplier;

/**
 * Triggers one-shot gamepad rumble events whenever a condition first becomes true.
 *
 * <p>Each registered condition is checked every loop via a rising-edge test: the event fires
 * exactly once when the condition transitions from false to true. It will not re-fire until
 * the condition goes false again and turns true a second time.</p>
 *
 * <h2>Match-timer events (pre-configured)</h2>
 * <ul>
 *   <li><b>90 s elapsed</b> - 2 blips (30 seconds left / Endgame warning)</li>
 *   <li><b>105 s elapsed</b> - 3 blips (15 seconds left / Final countdown)</li>
 *   <li><b>115 s elapsed</b> - 500 ms long pulse (5 seconds left / Match cutoff)</li>
 * </ul>
 *
 * <h2>Typical OpMode usage</h2>
 * <pre>
 *   // In init():
 *   rumble = new RumbleManager(gamepad1);
 *
 *   // In start():
 *   rumble.resetTimer();
 *
 *   // In loop():
 *   rumble.update();
 * </pre>
 *
 * <h2>Adding a custom event</h2>
 * <pre>
 *   rumble.addEvent(() -&gt; robot.isIntakeHolding(), () -&gt; gamepad1.rumbleBlips(1));
 * </pre>
 */
public class RumbleManager {

    // Maximum number of registered events (match events + user-defined slots).
    // Raise this constant if you add many custom events.
    private static final int MAX_EVENTS = 16;

    private final Gamepad gamepad;
    private final ElapsedTime matchTimer = new ElapsedTime();

    // Parallel arrays allocated once at construction - zero allocation during update().
    private final BooleanSupplier[] conditions    = new BooleanSupplier[MAX_EVENTS];
    private final Runnable[]        actions       = new Runnable[MAX_EVENTS];
    private final boolean[]         previousState = new boolean[MAX_EVENTS];
    private int eventCount = 0;

    /**
     * Creates a RumbleManager tied to one gamepad.
     *
     * @param gamepad the non-null gamepad that will receive rumble commands
     */
    public RumbleManager(Gamepad gamepad) {
        if (gamepad == null) {
            throw new IllegalArgumentException("RumbleManager requires a non-null gamepad.");
        }
        this.gamepad = gamepad;
        registerMatchTimerEvents();
    }

    /**
     * Resets the match timer to zero.
     *
     * <p>Call this from {@code start()} so match-time events are measured from the moment
     * the Driver Station presses PLAY, not from {@code init()}.</p>
     */
    public void resetTimer() {
        matchTimer.reset();
    }

    /**
     * Evaluates all registered conditions and fires any whose rising edge is detected.
     *
     * <p>Call exactly once per TeleOp {@code loop()}, after updating any sensor or
     * mechanism state that your custom conditions depend on.</p>
     */
    public void update() {
        for (int i = 0; i < eventCount; i++) {
            boolean current = conditions[i].getAsBoolean();
            if (current && !previousState[i]) {
                actions[i].run();
            }
            previousState[i] = current;
        }
    }

    /**
     * Adds a custom one-shot rumble event.
     *
     * <p>The {@code action} fires exactly once each time {@code condition} transitions from
     * false to true, then resets when the condition goes false again.</p>
     *
     * <pre>
     *   // Rumble when intake reaches Holding state:
     *   rumble.addEvent(() -> robot.getIntakeStateName().equals("Holding"), () -> gamepad1.rumbleBlips(1));
     *   // Rumble on a button press:
     *   rumble.addEvent(() -> gamepad1.a, () -> gamepad1.rumble(200));
     * </pre>
     *
     * @param condition returns {@code true} when the event should fire
     * @param action    the rumble command to run (e.g. {@code () -> gamepad1.rumbleBlips(2)})
     * @throws IllegalStateException if the event capacity is exceeded
     */
    public void addEvent(BooleanSupplier condition, Runnable action) {
        if (eventCount >= MAX_EVENTS) {
            throw new IllegalStateException(
                    "RumbleManager: cannot register more than " + MAX_EVENTS + " events. "
                    + "Raise MAX_EVENTS if you need more.");
        }
        conditions[eventCount]    = condition;
        actions[eventCount]       = action;
        previousState[eventCount] = false;
        eventCount++;
    }

    // -------------------------------------------------------------------------
    // Private helpers
    // -------------------------------------------------------------------------

    /** Registers the three standard FTC match-timer rumble events. */
    private void registerMatchTimerEvents() {
        // 90 s elapsed - 2 blips (30 s left, Endgame begins)
        addEvent(
            () -> matchTimer.seconds() >= 90.0,
            () -> gamepad.rumbleBlips(2)
        );

        // 105 s elapsed - 3 blips (15 s left, Final countdown)
        addEvent(
            () -> matchTimer.seconds() >= 105.0,
            () -> gamepad.rumbleBlips(3)
        );

        // 115 s elapsed - 500 ms long pulse (5 s left, Match cutoff imminent)
        addEvent(
            () -> matchTimer.seconds() >= 115.0,
            () -> gamepad.rumble(500)
        );
    }
}
