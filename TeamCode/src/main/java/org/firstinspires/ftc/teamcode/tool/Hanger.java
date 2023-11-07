package org.firstinspires.ftc.teamcode.tool;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.function.BooleanSupplier;

public class Hanger extends SubsystemBase {
    public static double leftServoStaticPos = 0.085D, leftServoUpPos = 0.5D, rightServoStaticPos = 0.545D, rightServoUpPos = 0.09D;
    private final Servo leftServo, rightServo;
    private final ToggleButtonReader toggle;

    private boolean lastVal;

    public Hanger(HardwareMap hardwareMap, BooleanSupplier upOrNot) {
        this.leftServo = hardwareMap.get(Servo.class, "leftHanger");
        this.rightServo = hardwareMap.get(Servo.class, "rightHanger");
        this.toggle = new ToggleButtonReader(upOrNot);
        periodic();
    }

    @Override
    public void periodic() {
        toggle.readValue();
        final boolean output = toggle.getState();
        if (output != lastVal) {
            leftServo.setPosition(output ? leftServoUpPos : leftServoStaticPos);
            rightServo.setPosition(output ? rightServoUpPos : rightServoStaticPos);
        }
        lastVal = output;
    }
}
