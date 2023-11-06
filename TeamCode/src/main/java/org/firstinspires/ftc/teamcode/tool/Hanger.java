package org.firstinspires.ftc.teamcode.tool;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.function.BooleanSupplier;

public class Hanger extends SubsystemBase {
    public static double leftServoStaticPos = 0D, leftServoUpPos = 0D, rightServoStaticPos = 0D, rightServoUpPos = 0D;
    private final Servo leftServo, rightServo;

    private BooleanSupplier up;
    private boolean lastVal;

    public Hanger(HardwareMap hardwareMap, BooleanSupplier upOrNot, boolean startingVal) {
        this.leftServo = hardwareMap.get(Servo.class, "leftHanger");
        this.rightServo = hardwareMap.get(Servo.class, "rightHanger");
        this.up = () -> startingVal; // don't ask
        lastVal = !startingVal;
        periodic();
        this.up = upOrNot;
    }

    public Hanger(HardwareMap hardwareMap, BooleanSupplier upOrNot) {
        this(hardwareMap, upOrNot, false);
    }

    @Override
    public void periodic() {
        final boolean output = up.getAsBoolean();
        if (output != lastVal) {
            leftServo.setPosition(output ? leftServoUpPos : leftServoStaticPos);
            rightServo.setPosition(output ? rightServoUpPos : rightServoStaticPos);
        }
        lastVal = output;
    }
}
