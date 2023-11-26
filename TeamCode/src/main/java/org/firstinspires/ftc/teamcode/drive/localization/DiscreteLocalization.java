package org.firstinspires.ftc.teamcode.drive.localization;


/**
 * DiscreteLocalization is not localization that runs continuously and accumulating
 * position deltas, rather it is localization that calculates your position based off of an input at
 * a single point in time.
 *
 * @see ContinuousLocalization
 * @see Localization
 * @see AprilTagLocalization
 */
public interface DiscreteLocalization extends Localization {
    /**
     * Returns the last time read as the number of milliseconds after the Unix epoch in UTC. (uses
     * {@link System#currentTimeMillis()}
     */
    long getLastReadTime();
}