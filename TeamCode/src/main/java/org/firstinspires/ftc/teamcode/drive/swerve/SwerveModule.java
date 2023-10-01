package org.firstinspires.ftc.teamcode.drive.swerve;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.hardware.AbsoluteAnalogEncoder;

import java.util.Locale;

@Config
public class SwerveModule {
    public enum Wheel {
        FRONT_LEFT(0D),
        FRONT_RIGHT(0D),
        BACK_LEFT(0D),
        BACK_RIGHT(0D);

        public final double tickOffset;

        Wheel(double tickOffset) {
            this.tickOffset = tickOffset;
        }
    }

    public static double P = 0.6, I = 0, D = 0.1;
    public static double K_STATIC = 0.03;

    public static double MAX_SERVO = 1, MAX_MOTOR = 1;

    public static double WHEEL_RADIUS = 1.41732; // TODO: MEASURE ACCURATELY
    public static final double GEAR_RATIO = 1 / ((42D / 12D) * (36D / 24D) * (2D)); // output (wheel) speed / input (motor) speed
    public static final double TICKS_PER_REV = 28;
    private final Wheel wheel;

    private final DcMotorEx motor;
    private final CRServo servo;
    private final AbsoluteAnalogEncoder encoder;
    private PIDFController rotationController;

    private double lastMotorPower = 0;
    private double target = 0.0;
    private double position = 0.0;

    public SwerveModule(DcMotorEx m, CRServo s, AbsoluteAnalogEncoder e, Wheel wheel) {
        motor = m;
        MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
        motorConfigurationType.setAchieveableMaxRPMFraction(MAX_MOTOR);
        motor.setMotorType(motorConfigurationType);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servo = s;
        ((CRServoImplEx) servo).setPwmRange(new PwmControl.PwmRange(500, 2500, 5000)); //TODO: figure out what to set framing rate to

        encoder = e;
        rotationController = new PIDFController(-1,-1,-1, 0);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.wheel = wheel;
    }

    public SwerveModule(@NonNull HardwareMap hardwareMap, String motorName, String servoName, String encoderName, Wheel wheel) {
        this(hardwareMap.get(DcMotorEx.class, motorName), hardwareMap.get(CRServo.class, servoName),
                new AbsoluteAnalogEncoder(hardwareMap.get(AnalogInput.class, encoderName)),
                wheel
        );
    }

    public void read() {
        position = encoder.getCurrentPosition() - wheel.tickOffset;
    }

    public void update(double p, double i, double d) {
        rotationController.setPIDF(p, i, d, 0);
        double target = getTargetRotation(), current = getModuleRotation();

        double error = normalizeRadians(target - current);

        error = normalizeRadians(target - current);
        double power = Range.clip(rotationController.calculate(0, error), -MAX_SERVO, MAX_SERVO);
        if (Double.isNaN(power)) power = 0;
        servo.setPower(power + (Math.abs(error) > 0.02 ? K_STATIC : 0) * Math.signum(power));
    }

    public double getTargetRotation() {
        return normalizeRadians(target - Math.PI);
    }

    public double getModuleRotation() {
        return normalizeRadians(position - Math.PI);
    }

    public void setMotorPower(double power) {
        lastMotorPower = power;
        motor.setPower(power);
    }

    public void setTargetRotation(double target) {
        this.target = normalizeRadians(target);
    }

    public void runTelemetry(@NonNull String name, @NonNull Telemetry telemetry) {
        final String caption = "module " + name;
        telemetry.addData(caption + " curr rotation", getModuleRotation());
        telemetry.addData(caption + " raw rotation", position);
        telemetry.addData(caption + " motor power", lastMotorPower);
        telemetry.addData(caption + " target rotation", getTargetRotation());
    }


    public void setMode(DcMotor.RunMode runMode) {
        motor.setMode(runMode);
    }


    public double getServoPower() {
        return servo.getPower();
    }

    public double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    public double getWheelPosition() {
        return encoderTicksToInches(motor.getCurrentPosition());
    }

    public double getWheelVelocity() {
        return encoderTicksToInches(motor.getVelocity());
    }

    public void setTargetModuleState(@NonNull SwerveModuleState state) {
        setMotorPower(state.speedMetersPerSecond);
        setTargetRotation(state.angle.getRadians());
    }
}
