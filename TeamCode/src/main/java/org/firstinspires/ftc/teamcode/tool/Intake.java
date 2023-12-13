package org.firstinspires.ftc.teamcode.tool;

import android.util.Pair;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.common.util.ConfigChangeDetector;
import org.firstinspires.ftc.teamcode.common.util.DcMotorSimpleGroup;

@Config
public class Intake extends ToggleableMotor {
    public static double SPEED = 0.9;
    private static final ConfigChangeDetector<Intake> changeDetector = new ConfigChangeDetector<>(Intake.class);
    public Intake(@NonNull HardwareMap hardwareMap) {
        super(new DcMotorSimpleGroup(
                new Pair<>(hardwareMap.get(DcMotor.class, "intake"), false),
                new Pair<>(hardwareMap.get(CRServo.class, "intakeServo"), true)
        ));
        changeDetector.update();
        setMaxMotorPower(SPEED);
    }

    @Override
    public void periodic() {
        if(changeDetector.updateAndHasChanged()) {
            setMaxMotorPower(SPEED);
        }
        super.periodic();
    }
}
