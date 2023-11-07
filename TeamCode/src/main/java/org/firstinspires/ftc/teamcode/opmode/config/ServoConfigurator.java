package org.firstinspires.ftc.teamcode.opmode.config;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class ServoConfigurator extends LinearOpMode {
    public static double servoPosition = 0D;
    public static String servoName = "";

    @Override
    public void runOpMode() {
        waitForStart();
        while (opModeIsActive()) {
            hardwareMap.get(Servo.class, servoName).setPosition(servoPosition);
        }
    }
}
