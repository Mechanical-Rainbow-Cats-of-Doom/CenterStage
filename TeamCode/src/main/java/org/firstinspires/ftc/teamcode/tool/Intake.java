package org.firstinspires.ftc.teamcode.tool;

import android.util.Pair;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.util.DcMotorSimpleGroup;

public class Intake extends ToggleableMotor {
    public Intake(@NonNull HardwareMap hardwareMap) {
        super(new DcMotorSimpleGroup(
                new Pair<>(hardwareMap.get(DcMotor.class, "intake"), false),
                new Pair<>(hardwareMap.get(CRServo.class, "intakeServo"), true)
        ));
    }
}
