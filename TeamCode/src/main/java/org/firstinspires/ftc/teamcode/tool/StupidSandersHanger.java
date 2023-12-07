package org.firstinspires.ftc.teamcode.tool;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.function.BooleanSupplier;

@Config(value = "StupidSandersHanger Servo Positions")
public class StupidSandersHanger extends SubsystemBase {
    public static double leftServoStaticPos = 0.085D, leftServoUpPos = 0.5D, rightServoStaticPos = 0.545D, rightServoUpPos = 0.09D;
    private final ServoToggle left, right;
    private final ToggleButtonReader toggle;

    public StupidSandersHanger(HardwareMap hardwareMap, BooleanSupplier upOrNot) {
        this.left = new ServoToggle(hardwareMap, "leftHanger", leftServoStaticPos, leftServoUpPos);
        this.right = new ServoToggle(hardwareMap, "rightHanger", rightServoStaticPos, rightServoUpPos);
        this.toggle = new ToggleButtonReader(upOrNot);
    }

    @Override
    public void periodic() {
        toggle.readValue();
        final boolean output = toggle.getState();
        left.setState(output);
        right.setState(output);
        left.update();
        right.update();
    }
}
