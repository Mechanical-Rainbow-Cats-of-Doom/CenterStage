package org.firstinspires.ftc.teamcode.opmode.auto;

import android.util.Pair;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.hardware.AbsoluteAnalogEncoder;
import org.firstinspires.ftc.teamcode.common.util.DcMotorSimpleGroup;
import org.firstinspires.ftc.teamcode.drive.swerve.SwerveModule;
import org.firstinspires.ftc.teamcode.vision.PropDetector;
import org.firstinspires.ftc.teamcode.vision.PropPipeline;

/**
 * All units are in arbitrary ticks
 */
@Disabled
@Autonomous
@Config
public class stinkyBackup extends LinearOpMode {
    public static double turnStrength = 0; // heading correction makes things worse
    public static int turnLeftDistance = 6500;
    @Override
    public void runOpMode() throws InterruptedException {
        MultipleTelemetry telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());
        ElapsedTime timer = new ElapsedTime();

        backRight =
                new Pod("backRightMotor", "backRightServo", "backRightEncoder",
                        false, 1, SwerveModule.Wheel.BACK_RIGHT);
        backLeft =
                new Pod("backLeftMotor", "backLeftServo", "backLeftEncoder",
                        false, 1, SwerveModule.Wheel.BACK_LEFT);
        frontRight =
                new Pod("frontRightMotor", "frontRightServo", "frontRightEncoder",
                        false, 1, SwerveModule.Wheel.FRONT_RIGHT);
        frontLeft =
                new Pod("frontLeftMotor", "frontLeftServo", "frontLeftEncoder",
                        true, 1, SwerveModule.Wheel.FRONT_LEFT);
        DcMotorSimpleGroup intake = new DcMotorSimpleGroup(
                new Pair<>(hardwareMap.get(DcMotor.class, "intake"), false),
                new Pair<>(hardwareMap.get(CRServo.class, "intakeServo"), true)
        );
//        intake = new DcMotorEx

        // fin vision here
        PropDetector detector = new PropDetector(hardwareMap, "webcam", true,
                true, PropPipeline.PropPipelineRectsProvider.Default.DEFAULT
        );
        int result = -1;
        float startTime = System.currentTimeMillis() / 1000f;

//        OldIntake intake = new OldIntake(hardwareMap);

        waitForStart();

        result = detector.run(() -> {
            int time = (int)((System.currentTimeMillis() - startTime) / 10f) % 4;
            telemetry.addLine("Waiting for detector" + (time > 1 ? "." : "") +
                    (time > 2 ? "." : "") +
                    (time > 3 ? "." : ""));
            telemetry.update();
        });
        detector.reset();

        switch (result) {
            case -1: // shouldn't happen
                return;
            case 0: // left
                // move to left line
                telemetry.addLine("turning");
                telemetry.update();
                turnLeft(turnLeftDistance, 1);
                turnRight(-3000,1);
                turnLeft(1000, 1);
                // outtake
                intake.setPower(-0.8);
                timer.reset();
                while (timer.seconds() < 1 && opModeIsActive() && !isStopRequested()) {
                    continue;
                    //lmao funny wait
                }
                turnRight(-1000,1);
                intake.setPower(0);
                break;
            case 1: // middle
                // move to line
                forward(1000, 0.8);
                // outtake
                intake.setPower(-1);
                timer.reset();
                while (timer.seconds() < 2 && opModeIsActive() && !isStopRequested()) {
                    continue;
                    //lmao funny wait
                }
                forward(-200, 0.3);
                intake.setPower(0);
                break;
            case 2: // right
                // move to right line
                telemetry.addLine("turning");
                telemetry.update();
                turnRight(7000, 1);
                turnLeft(-2000,1);
                turnRight(2000, 1);
                // outtake
                intake.setPower(-1);
                timer.reset();
                while (timer.seconds() < 2 && opModeIsActive() && !isStopRequested()) {
                    continue;
                    //lmao funny wait
                }
                turnLeft(-1000,1);
                intake.setPower(0);
                break;

        }
    }

    /*
    Separate
    Separate
    Separate
    Separate
    Separate
    Separate
    Separate
    Separate
    Separate
    Separate
    Separate
    Separate
    Separate
    Separate
    Separate
    Separate
    Separate
    Separate
    Separate
    Separate
    Separate
    Separate
    Separate
    Separate
    Separate
     */
    private DcMotorEx intake;
    private Pod backRight;
    private Pod backLeft;
    private Pod frontRight;
    private Pod frontLeft;
    public void forward(int distance, double powerMultiplier) {
        frontLeft.setDistance(distance);
        frontRight.setDistance(distance);
        backLeft.setDistance(distance);
        backRight.setDistance(distance);
        while (!frontLeft.isComplete() && !frontRight.isComplete()
                && !backLeft.isComplete() && !backRight.isComplete()
                && opModeIsActive() && !isStopRequested()) {
            frontRight.run(powerMultiplier);
            frontLeft.run(powerMultiplier);
            backLeft.run(powerMultiplier);
            backRight.run(powerMultiplier);
        }
    }
    public void forwardTest(double power) {
        frontRight.runTest(power);
        frontLeft.runTest(power);
        backLeft.runTest(power);
        backRight.runTest(power);
    }

    public void turn(int distance, double powerMultiplier) {
        frontLeft.setDistance(distance);
        frontRight.setDistance(-distance);
        backLeft.setDistance(distance);
        backRight.setDistance(-distance);
        while (!frontLeft.isComplete() && !frontRight.isComplete()
                && !backLeft.isComplete() && !backRight.isComplete()
                && opModeIsActive() && !isStopRequested()) {
            frontRight.run(powerMultiplier);
            frontLeft.run(powerMultiplier);
            backLeft.run(powerMultiplier);
            backRight.run(powerMultiplier);
        }
        frontRight.stop();
        frontLeft.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void turnRight(int distance, double powerMultiplier) {
        frontLeft.setDistance(distance);
        frontRight.setDistance((int) Math.ceil(distance*0.1));
        backLeft.setDistance(distance);
        backRight.setDistance((int) Math.ceil(distance*0.1));
        while (!frontLeft.isComplete() && !frontRight.isComplete()
                && !backLeft.isComplete() && !backRight.isComplete()
                && opModeIsActive() && !isStopRequested()) {
            frontRight.run(powerMultiplier*0.1);
            frontLeft.run(powerMultiplier);
            backLeft.run(powerMultiplier);
            backRight.run(powerMultiplier*0.1);
        }
        frontRight.stop();
        frontLeft.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void turnLeft(int distance, double powerMultiplier) {
        frontLeft.setDistance((int) Math.ceil(distance*0.1));
        frontRight.setDistance(distance);
        backLeft.setDistance((int) Math.ceil(distance*0.1));
        backRight.setDistance(distance);
        while (!frontLeft.isComplete() && !frontRight.isComplete()
                && !backLeft.isComplete() && !backRight.isComplete()
                && opModeIsActive() && !isStopRequested()) {
            frontRight.run(powerMultiplier);
            frontLeft.run(powerMultiplier*0.1);
            backLeft.run(powerMultiplier*0.1);
            backRight.run(powerMultiplier);
        }
        frontRight.stop();
        frontLeft.stop();
        backLeft.stop();
        backRight.stop();
    }

    public class Pod {
        /*
        ADD IN A WAY TO KEEP THE SERVO CENTERED, A SHITTY TURNING SOLUTION IS CHILL
         */
        public double multiplier;
        int tickOffset = 0;
        int tickGoal = 0;
        double initialServoPosition;
        MotorEx wheel;
        CRServo servo;
        AbsoluteAnalogEncoder encoder;
        SwerveModule.Wheel whichWheel;
        Pod(String motorName, String servoName, String encoderName, boolean inverted, double multiplier, SwerveModule.Wheel whichWheel) {
            wheel = new MotorEx(hardwareMap, motorName, Motor.GoBILDA.BARE);
            wheel.setRunMode(Motor.RunMode.VelocityControl);
            wheel.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            wheel.setInverted(inverted);
            servo = hardwareMap.get(CRServo.class, servoName);
            encoder = new AbsoluteAnalogEncoder(hardwareMap.get(AnalogInput.class, encoderName));
            this.multiplier = multiplier;
            tickOffset = wheel.getCurrentPosition();
            initialServoPosition = encoder.getCurrentPosition();
            this.whichWheel = whichWheel;
            SwerveModule.wheelAutoPositionMap.put(whichWheel, initialServoPosition);
        }
        public double getServoPosition() {
            return encoder.getCurrentPosition()-initialServoPosition;
        }
        public boolean isComplete() {
            return Math.signum(tickGoal)*(tickGoal - (wheel.getCurrentPosition() - tickOffset)) <= 0;
        }
        public void runTest(double power) {
            wheel.set(power);
        }
        public void run(double powerMultiplier) {

            if (Math.signum(tickGoal)*(tickGoal - (wheel.getCurrentPosition() - tickOffset)) > 0) {
                wheel.set(powerMultiplier*Math.signum(tickGoal));
            } else wheel.set(0);
        }
        public void stop() {
            wheel.set(0);
        }
        public void setDistance(int distance) {
            tickOffset = wheel.getCurrentPosition();
            tickGoal = (int) Math.ceil(distance*multiplier);
        }
    }
}
