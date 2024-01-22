package org.firstinspires.ftc.teamcode.drive.mecanum;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.drivebase.RobotDrive;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.HolonomicDrive;

import java.util.function.BooleanSupplier;

@Config
public class MecanumDriveSubsystem extends SubsystemBase implements HolonomicDrive {
    public static double FL_MULTIPLIER = 1;
    public static double FR_MULTIPLIER = 1;
    public static double BL_MULTIPLIER = 1;
    public static double BR_MULTIPLIER = 1;

    @SuppressWarnings("FieldCanBeLocal")
    private final MotorEx frontLeft, frontRight, backLeft, backRight;
    private final Telemetry telemetry;

    private final MecanumDrive drive;
    private final IMU imu;
    private final boolean teleOp;
    private final BooleanSupplier fieldOriented;

    private ChassisSpeeds currVelocity = new ChassisSpeeds();

    public MecanumDriveSubsystem(HardwareMap hardwareMap, BooleanSupplier fieldOriented, boolean teleOp,
                                 Telemetry telemetry) {
        this.frontLeft = new MotorEx(hardwareMap, "frontLeftMotor");
        this.frontLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontLeft.setInverted(true);
        this.frontRight = new MotorEx(hardwareMap, "frontRightMotor");
        this.frontRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontRight.setInverted(true);
        this.backLeft = new MotorEx(hardwareMap, "backLeftMotor");
        this.backLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.backRight = new MotorEx(hardwareMap, "backRightMotor");
        this.backRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.drive = new MecanumDrive(true, frontLeft, frontRight, backLeft, backRight);
        this.imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
        )));
        imu.resetYaw();

        this.teleOp = teleOp;
        this.fieldOriented = fieldOriented;
        this.telemetry = telemetry;
    }

    @Override
    public void setTargetVelocity(ChassisSpeeds targetVelocity) {
        this.currVelocity = targetVelocity;
    }


    @Override
    public void periodic() {
        if (fieldOriented.getAsBoolean()) {
            this.drive.driveFieldCentric(currVelocity.vyMetersPerSecond, currVelocity.vxMetersPerSecond, currVelocity.omegaRadiansPerSecond, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES), teleOp);
        } else {
//            double strafeSpeed = drive.clipRange(currVelocity.vxMetersPerSecond);
//            double forwardSpeed = drive.clipRange(currVelocity.vyMetersPerSecond);
//            double turnSpeed = drive.clipRange(currVelocity.omegaRadiansPerSecond);
//
//            Vector2d input = new Vector2d(strafeSpeed, forwardSpeed);
//
//            double theta = input.angle();
//
//            double[] wheelSpeeds = new double[4];
//            wheelSpeeds[RobotDrive.MotorType.kFrontLeft.value] = Math.sin(theta + Math.PI / 4);
//            wheelSpeeds[RobotDrive.MotorType.kFrontRight.value] = Math.sin(theta - Math.PI / 4);
//            wheelSpeeds[RobotDrive.MotorType.kBackLeft.value] = Math.sin(theta - Math.PI / 4);
//            wheelSpeeds[RobotDrive.MotorType.kBackRight.value] = Math.sin(theta + Math.PI / 4);
//
//            normalize(wheelSpeeds, input.magnitude());
//
//            wheelSpeeds[RobotDrive.MotorType.kFrontLeft.value] += turnSpeed;
//            wheelSpeeds[RobotDrive.MotorType.kFrontRight.value] -= turnSpeed;
//            wheelSpeeds[RobotDrive.MotorType.kBackLeft.value] += turnSpeed;
//            wheelSpeeds[RobotDrive.MotorType.kBackRight.value] -= turnSpeed;
//
//            normalize(wheelSpeeds);
//
//            this.drive.driveWithMotorPowers(
//                    wheelSpeeds[RobotDrive.MotorType.kFrontLeft.value] * FL_MULTIPLIER,
//                    wheelSpeeds[RobotDrive.MotorType.kFrontRight.value] * FR_MULTIPLIER,
//                    wheelSpeeds[RobotDrive.MotorType.kBackLeft.value] * BL_MULTIPLIER,
//                    wheelSpeeds[RobotDrive.MotorType.kBackRight.value] * BR_MULTIPLIER
//            );
            drive.driveRobotCentric(currVelocity.vyMetersPerSecond, currVelocity.vxMetersPerSecond, currVelocity.omegaRadiansPerSecond);
        }
    }

    protected void normalize(double[] wheelSpeeds) {
        normalize(wheelSpeeds, 1);
    }

    protected void normalize(double[] wheelSpeeds, double magnitude) {
        double maxMagnitude = Math.abs(wheelSpeeds[0]);
        for (int i = 1; i < wheelSpeeds.length; i++) {
            double temp = Math.abs(wheelSpeeds[i]);
            if (maxMagnitude < temp) {
                maxMagnitude = temp;
            }
        }
        for (int i = 0; i < wheelSpeeds.length; i++) {
            wheelSpeeds[i] = (wheelSpeeds[i] / maxMagnitude) * magnitude;
        }
    }
}
