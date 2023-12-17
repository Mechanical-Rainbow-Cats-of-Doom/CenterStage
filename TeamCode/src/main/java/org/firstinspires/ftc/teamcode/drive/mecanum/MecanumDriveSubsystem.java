package org.firstinspires.ftc.teamcode.drive.mecanum;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.HolonomicDrive;

import java.util.function.BooleanSupplier;

import kotlin.NotImplementedError;

public class MecanumDriveSubsystem extends SubsystemBase implements HolonomicDrive {
    @SuppressWarnings("FieldCanBeLocal")
    private final MotorEx frontLeft, frontRight, backLeft, backRight;

    private final MecanumDrive drive;
    private final IMU imu;
    private final boolean teleOp;
    private final BooleanSupplier fieldOriented;

    private ChassisSpeeds currVelocity;

    public MecanumDriveSubsystem(HardwareMap hardwareMap, BooleanSupplier fieldOriented, boolean teleOp) {
        this.frontLeft = new MotorEx(hardwareMap, "frontLeftMotor");
        this.frontLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        this.frontRight = new MotorEx(hardwareMap, "frontRightMotor");
        this.frontRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
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
            this.drive.driveRobotCentric(currVelocity.vyMetersPerSecond, currVelocity.vxMetersPerSecond, currVelocity.omegaRadiansPerSecond, teleOp);
        }
    }
}
