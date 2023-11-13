package org.firstinspires.ftc.teamcode.tool;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class DroneLauncher extends SubsystemBase {
    public static double HELD_POS = 0.0, LAUNCHED_POS = 0.0;
    private final ServoToggle rubberBandHolder;

    public DroneLauncher(HardwareMap hardwareMap) {
        this.rubberBandHolder = new ServoToggle(hardwareMap, "drone", HELD_POS, LAUNCHED_POS);
    }

    @Override
    public void periodic() {
        rubberBandHolder.update();
    }
}
