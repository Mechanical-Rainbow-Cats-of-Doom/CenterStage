package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Test Motor or Servo")
@Config
public class TestMotor extends LinearOpMode {
    public static String motorOrServoName = "";
    public static double power = 0D;
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        while (opModeIsActive()) {
            hardwareMap.get(DcMotorSimple.class, motorOrServoName).setPower(power);
        }
    }
}
