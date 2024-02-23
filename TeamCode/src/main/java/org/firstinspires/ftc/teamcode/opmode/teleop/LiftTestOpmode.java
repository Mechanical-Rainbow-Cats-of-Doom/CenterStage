package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp
public class LiftTestOpmode extends LinearOpMode {
    public static double LIFT_POWER = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        MotorEx liftLeft = new MotorEx(hardwareMap, "liftLeft", Motor.GoBILDA.RPM_312);
        MotorEx liftRight = new MotorEx(hardwareMap, "liftRight", Motor.GoBILDA.RPM_312);
        MultipleTelemetry telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());

        liftLeft.setRunMode(Motor.RunMode.RawPower);
        liftRight.setRunMode(Motor.RunMode.RawPower);

        GamepadEx driver1 = new GamepadEx(gamepad1);

        while(!isStopRequested()) {
            liftLeft.set(driver1.getLeftY()*LIFT_POWER);
            liftRight.set(driver1.getRightY()*LIFT_POWER);

            telemetry.addData("Right Lift Position: ", liftRight.getCurrentPosition());

            telemetry.addData("Left Stick Y", driver1.getLeftY());
            telemetry.addData("Right Stick Y", driver1.getRightY());
            telemetry.update();
        }
    }
}
