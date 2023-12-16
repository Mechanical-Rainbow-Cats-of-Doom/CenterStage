package org.firstinspires.ftc.teamcode.opmode.auto;

import android.util.Pair;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.hardware.AbsoluteAnalogEncoder;
import org.firstinspires.ftc.teamcode.drive.swerve.SwerveModule;
import org.firstinspires.ftc.teamcode.tool.Intake;
import org.firstinspires.ftc.teamcode.vision.PropDetector;
import org.firstinspires.ftc.teamcode.vision.PropPipeline;

/**
 * All units are in arbitrary ticks
 */
@Autonomous
@Config
public class stinkyBackup extends LinearOpMode {
    public static double turnStrength = 0; // heading correction makes things worse
    @Override
    public void runOpMode() throws InterruptedException {
        MultipleTelemetry telemetry = new MultipleTelemetry(super.telemetry, FtcDashboard.getInstance().getTelemetry());
        ElapsedTime timer = new ElapsedTime();

        backRight = new pod("backRightMotor", "backRightServo", "backRightEncoder",
                false, 1, turnStrength, SwerveModule.Wheel.BACK_RIGHT);
        backLeft =
                new pod("backLeftMotor", "backLeftServo", "backLeftEncoder",
                        false, 1, turnStrength, SwerveModule.Wheel.BACK_LEFT);
        frontRight =
                new pod("frontRightMotor", "frontRightServo", "frontRightEncoder",
                        false, 1, turnStrength, SwerveModule.Wheel.FRONT_RIGHT);
        frontLeft =
                new pod("frontLeftMotor", "frontLeftServo", "frontLeftEncoder",
                        true, 1, turnStrength, SwerveModule.Wheel.FRONT_LEFT);

//        intake = new DcMotorEx

        // fin vision here
//        PropDetector detector = new PropDetector(hardwareMap, "webcam", true,
//                true, new PropPipeline.PropPipelineRectsProvider.PropPipelineDashboardConfig()
//        );
        int result = -1;
        float startTime = System.currentTimeMillis() / 1000f;

        Intake intake = new Intake(hardwareMap);

        waitForStart();

//        result = detector.run(() -> {
//            int time = (int)((System.currentTimeMillis() - startTime) / 10f) % 4;
//            telemetry.addLine("Waiting for detector" + (time > 1 ? "." : "") +
//                    (time > 2 ? "." : "") +
//                    (time > 3 ? "." : ""));
//            telemetry.update();
//        });
//        detector.reset();
        result = 0;
        // get to middle of the tile in front
        forward(1000, 0.2);
        switch (result) {
            case -1: // shouldn't happen
                return;
            case 0: // left
                // turn 90 degrees
                turn(300, 0.2);
                // move to line
                forward(200, 0.2);
                // outtake
                intake.toggleOn();
                intake.toggleState();
                timer.reset();
                while (timer.seconds() < 2 && opModeIsActive() && !isStopRequested()) {
                    continue;
                    //lmao funny wait
                }
                intake.toggleOn();
                break;
            case 1: // middle
                // move to line
                forward(200, 0.2);
                // outtake
                intake.toggleOn();
                intake.toggleState();
                timer.reset();
                while (timer.seconds() < 2 && opModeIsActive() && !isStopRequested()) {
                    continue;
                    //lmao funny wait
                }
                intake.toggleOn();
                break;
            case 2: // right
                // turn 90 degrees
                turn(300, 0.2);
                // move to line
                forward(200, 0.2);
                // outtake
                intake.toggleOn();
                intake.toggleState();
                timer.reset();
                while (timer.seconds() < 2 && opModeIsActive() && !isStopRequested()) {
                    continue;
                    //lmao funny wait
                }
                intake.toggleOn();
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
    private pod backRight;
    private pod backLeft;
    private pod frontRight;
    private pod frontLeft;
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
            frontRight.run(power);
            frontLeft.run(power);
            backLeft.run(power);
            backRight.run(power);
        }
    }
    public class pod {
        /*
        ADD IN A WAY TO KEEP THE SERVO CENTERED, A SHITTY TURNING SOLUTION IS CHILL
         */
        public double multiplier;
        int tickOffset = 0;
        int tickGoal = 0;
        double initialServoPosition;
        double turnStrength;
        MotorEx wheel;
        CRServo servo;
        AbsoluteAnalogEncoder encoder;
        SwerveModule.Wheel whichWheel;
        pod(String motorName, String servoName, String encoderName, boolean inverted, double multiplier, double turnStrength, SwerveModule.Wheel whichWheel) {
            wheel = new MotorEx(hardwareMap, motorName, Motor.GoBILDA.BARE);
            wheel.setRunMode(Motor.RunMode.VelocityControl);
            wheel.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            wheel.setInverted(inverted);
            servo = hardwareMap.get(CRServo.class, servoName);
            encoder = new AbsoluteAnalogEncoder(hardwareMap.get(AnalogInput.class, encoderName));
            this.multiplier = multiplier;
            tickOffset = wheel.getCurrentPosition();
            initialServoPosition = encoder.getCurrentPosition();
            this.turnStrength = turnStrength;
            this.whichWheel = whichWheel;
        }
        public double getServoPosition() {
            return encoder.getCurrentPosition();
        }
        public boolean isComplete() {
            return wheel.getCurrentPosition() - tickOffset >= tickGoal;
        }
        public void run(double power) {
            if (getServoPosition() - initialServoPosition > 0) {
                servo.setPower(-turnStrength);
            } else if (getServoPosition() - initialServoPosition < 0) {
                servo.setPower(turnStrength);
            } else servo.setPower(0);

            if ((tickGoal - (wheel.getCurrentPosition() - tickOffset)) > 0) {
                wheel.set(power);
            } else wheel.set(0);
        }
        public void setDistance(int distance) {
            tickGoal = (int) Math.ceil(distance*multiplier);
        }
    }
}
