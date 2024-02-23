package org.firstinspires.ftc.teamcode.opmode.config;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class DualServoConfigurator extends LinearOpMode {
    public static double servoPosition = 0D;
    public static String firstServoName = "";
    public static String secondServoName = "";
    public static double firstServoMiddle = 0.5;
    public static double secondServoMiddle = 0.46;
    public static boolean secondServoInverted = true;

    @Override
    public void runOpMode() {
        waitForStart();
        Servo firstServo = hardwareMap.get(Servo.class, firstServoName);
        Servo secondServo = hardwareMap.get(Servo.class, secondServoName);
        while (opModeIsActive()) {
            firstServo.setPosition(servoPosition);
            secondServo.setPosition((servoPosition-firstServoMiddle)*(secondServoInverted ? -1 : 1)+secondServoMiddle);
        }
    }
}
