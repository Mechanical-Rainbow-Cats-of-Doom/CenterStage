package org.firstinspires.ftc.teamcode.tool;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.function.BooleanSupplier;

@Config
public class DroneLauncher extends SubsystemBase {
    public static double HELD_POS = 1D, LAUNCHED_POS = 0D;
    private final ServoToggle rubberBandHolder;
    private final ButtonReader reader;

    public DroneLauncher(HardwareMap hardwareMap, BooleanSupplier button) {
        this.rubberBandHolder = new ServoToggle(hardwareMap, "drone", HELD_POS, LAUNCHED_POS);
        this.reader = new ButtonReader(button);
    }

    @Override
    public void periodic() {
        reader.readValue();
        if (reader.wasJustReleased()) {
            rubberBandHolder.toggleState();
        }
        rubberBandHolder.update();
    }
}
