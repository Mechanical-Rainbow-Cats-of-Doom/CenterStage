package org.firstinspires.ftc.teamcode.drive.mecanum;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.HolonomicDrive;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

import java.util.function.BooleanSupplier;

@Config
public class MecanumDriveSubsystem extends SubsystemBase implements HolonomicDrive {
    public static double FL_MULTIPLIER = 1;
    public static double FR_MULTIPLIER = 1;
    public static double BL_MULTIPLIER = 1;
    public static double BR_MULTIPLIER = 1;

    @SuppressWarnings("FieldCanBeLocal")
//    private final MotorEx frontLeft, frontRight, backLeft, backRight;
    private final Telemetry telemetry;

    private final MecanumDrive drive;
    private final IMU imu;
    private final boolean teleOp;
    private final BooleanSupplier fieldOriented;
    private final boolean squareInputs;

    private ChassisSpeeds currVelocity = new ChassisSpeeds();

    public MecanumDriveSubsystem(HardwareMap hardwareMap, BooleanSupplier fieldOriented, boolean teleOp,
                                 Telemetry telemetry, boolean squareInputs) {
//        this.frontLeft = new MotorEx(hardwareMap, "frontLeftMotor");
//        this.frontLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
//        this.frontRight = new MotorEx(hardwareMap, "frontRightMotor");
//        this.frontRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
//        this.backLeft = new MotorEx(hardwareMap, "backLeftMotor");
//        this.backLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
//        this.backRight = new MotorEx(hardwareMap, "backRightMotor");
//        this.backRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.drive = new MecanumDrive(hardwareMap, new Pose2d(0,0,0));
        this.imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
        )));
        imu.resetYaw();

        this.teleOp = teleOp;
        this.fieldOriented = fieldOriented;
        this.telemetry = telemetry;
        this.squareInputs = squareInputs;
    }

    @Override
    public void setTargetVelocity(ChassisSpeeds targetVelocity) {
        this.currVelocity = targetVelocity;
    }


    @Override
    public void periodic() {
        if(squareInputs) {
            currVelocity.vxMetersPerSecond = Math.signum(currVelocity.vxMetersPerSecond)*currVelocity.vxMetersPerSecond*currVelocity.vxMetersPerSecond;
            currVelocity.vyMetersPerSecond = Math.signum(currVelocity.vyMetersPerSecond)*currVelocity.vyMetersPerSecond*currVelocity.vyMetersPerSecond;
            currVelocity.omegaRadiansPerSecond = Math.signum(currVelocity.omegaRadiansPerSecond)*currVelocity.omegaRadiansPerSecond*currVelocity.omegaRadiansPerSecond;
        }
        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(currVelocity.vxMetersPerSecond, -currVelocity.omegaRadiansPerSecond),
                currVelocity.vyMetersPerSecond));
//        if (fieldOriented.getAsBoolean()) {
//            drive.driveFieldCentric(currVelocity.vyMetersPerSecond, currVelocity.vxMetersPerSecond, currVelocity.omegaRadiansPerSecond, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES), teleOp);
//        } else {
//            drive.driveRobotCentric(currVelocity.vyMetersPerSecond, currVelocity.vxMetersPerSecond, currVelocity.omegaRadiansPerSecond);
//        }
    }
}
