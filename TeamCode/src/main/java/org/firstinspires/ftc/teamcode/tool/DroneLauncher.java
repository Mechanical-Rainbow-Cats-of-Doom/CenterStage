package org.firstinspires.ftc.teamcode.tool;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.BooleanSupplier;

@Config
public class DroneLauncher extends SubsystemBase {
    public static double HELD_POS = 0.54D, LAUNCHED_POS = 0.65D;
    public static double WAIT_TIME = 1;
    public static DroneLauncher instance;
    private final ServoToggle rubberBandHolder;
    public final ButtonReader reader;
    private final ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public DroneLauncher(HardwareMap hardwareMap, BooleanSupplier button) {
        this.rubberBandHolder = new ServoToggle(hardwareMap, "drone", HELD_POS, LAUNCHED_POS);
        this.reader = new ButtonReader(button);
        instance = this;
    }

    @Override
    public void periodic() {
        reader.readValue();
        if (rubberBandHolder.getState()) {
            if (timer.seconds() > WAIT_TIME) {
                rubberBandHolder.toggleState();
            }
            return;
        }
        if (reader.wasJustReleased()) {
            rubberBandHolder.toggleState();
            if (rubberBandHolder.getState()) {
                timer.reset();
            }
        }
        rubberBandHolder.update();
    }
}
