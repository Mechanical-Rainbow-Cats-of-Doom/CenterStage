package org.firstinspires.ftc.teamcode.common.util;

import com.arcrobotics.ftclib.util.Timing;

import java.util.concurrent.TimeUnit;

public class DelayStorage {
    public static double seconds = 0;

    public static void setSeconds(double seconds) {
        DelayStorage.seconds = seconds;
    }

    public static void addSeconds(double seconds) {
        DelayStorage.seconds += seconds;
    }

    public static void subtractSeconds(double seconds) {
        DelayStorage.seconds = Math.max(DelayStorage.seconds - seconds, 0);
    }

    public static Timing.Timer getTimer() {
        return new Timing.Timer((long)(seconds*1000), TimeUnit.MILLISECONDS);
    }

    /**
     * Waits for the delay to be elapsed.
     */
    public static void waitForDelay() {
        try {
            Thread.sleep((long) Math.ceil(seconds * 1000D));
        } catch (InterruptedException ignored) {}
    }
}
