package org.firstinspires.ftc.teamcode.opmode.auto;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.common.hardware.AbsoluteAnalogEncoder;

/**
 * All units are in arbitrary ticks
 */
@Autonomous
public class stinkyBackup extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        forward(100, 0.2);
    }

    /*
    Seperate
    Seperate
    Seperate
    Seperate
    Seperate
    Seperate
    Seperate
    Seperate
    Seperate
    Seperate
    Seperate
    Seperate
    Seperate
    Seperate
    Seperate
    Seperate
    Seperate
    Seperate
    Seperate
    Seperate
    Seperate
    Seperate
    Seperate
    Seperate
    Seperate
     */
    private final pod backRight =
            new pod("backRightMotor", "backRightServo", "backRightEncoder",
                    false, 1);
    private final pod backLeft =
            new pod("backLeftMotor", "backLeftServo", "backLeftEncoder",
                    false, 1);
    private final pod frontRight =
            new pod("frontRightMotor", "frontRightServo", "frontRightEncoder",
                    false, 1);
    private final pod frontLeft =
            new pod("frontLeftMotor", "frontLeftServo", "frontLeftEncoder",
                    true, 1);
    public void forward(int distance, double power) {
        frontLeft.setDistance(distance);
        frontRight.setDistance(distance);
        backLeft.setDistance(distance);
        backRight.setDistance(distance);
        while (!frontLeft.isComplete() && !frontRight.isComplete()
                && !backLeft.isComplete() && !backRight.isComplete()) {
            frontRight.run(power);
            frontLeft.run(power);
            backLeft.run(power);
            backRight.run(power);
        }
    }
    public void turn(int distance, double power) {
        frontLeft.setDistance(distance);
        frontRight.setDistance(-distance);
        backLeft.setDistance(distance);
        backRight.setDistance(-distance);
        while (!frontLeft.isComplete() && !frontRight.isComplete()
                && !backLeft.isComplete() && !backRight.isComplete()) {
            frontRight.run(-power);
            frontLeft.run(power);
            backLeft.run(power);
            backRight.run(-power);
        }
    }
    public class pod {
        public double multiplier;
        int tickOffset = 0;
        int tickGoal = 0;
        MotorEx wheel;
        CRServo servo;
        AbsoluteAnalogEncoder encoder;
        pod(String motorName, String servoName, String encoderName, boolean inverted, double multiplier) {
            wheel = new MotorEx(hardwareMap, motorName, Motor.GoBILDA.BARE);
            wheel.setRunMode(Motor.RunMode.VelocityControl);
            wheel.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            wheel.setInverted(inverted);
            servo = hardwareMap.get(CRServo.class, servoName);
            encoder = new AbsoluteAnalogEncoder(hardwareMap.get(AnalogInput.class, encoderName));
            this.multiplier = multiplier;
            tickOffset = wheel.getCurrentPosition();
        }
        public boolean isComplete() {
            return wheel.getCurrentPosition() - tickOffset >= tickGoal;
        }
        public void run(double power) {
            servo.setPower(0);
            if ((tickGoal - (wheel.getCurrentPosition() - tickOffset)) > 0) {
                wheel.set(power);
            } else wheel.set(0);
        }
        public void setDistance(int distance) {
            tickGoal = (int) Math.ceil(distance*multiplier);
        }
    }
}
