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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.hardware.AbsoluteAnalogEncoder;

@Config
public class SwerveModule {
    public enum Wheel {
        FRONT_LEFT(DcMotorSimple.Direction.REVERSE),
        FRONT_RIGHT(DcMotorSimple.Direction.FORWARD),
        BACK_LEFT(DcMotorSimple.Direction.FORWARD),
        BACK_RIGHT(DcMotorSimple.Direction.FORWARD);

        public final DcMotorSimple.Direction direction;

        Wheel(DcMotorSimple.Direction direction) {
            this.direction = direction;
        }
    }

    public static double FLIP_GAP = 1.1;
    public static double MAX_SERVO = 1, MAX_MOTOR = 1;

    public static final double EPSILON = 1e-5;
    public static double WHEEL_RADIUS = 1.41732; // TODO: MEASURE ACCURATELY
    public static final double MOTOR_GEAR_RATIO = 1 / ((42D / 12D) * (36D / 24D) * (2D)); // output (wheel) speed / input (motor) speed
    public static final double TICKS_PER_REV = 28;
    public static double POD_GEAR_RATIO = 1.0;

    private final Wheel wheel;

    private final DcMotorEx motor;
    private final CRServo servo;
    private final AbsoluteAnalogEncoder encoder;
    private final PIDFController rotationController;
    private final double dynamicTickOffset;

    private double lastMotorPower = 0;
    private double lastRotationTarget = 0D;
    private double target = 0.0;
    private double outputTarget = 0D;
    private double position = 0.0;
    private double power = 0D;
    private boolean flip = false;
    private double error = 0;

    public SwerveModule(DcMotorEx m, CRServo s, AbsoluteAnalogEncoder e, Wheel wheel) {
        motor = m;
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(wheel.direction);

        try {
            Thread.sleep(100);
        } catch (InterruptedException ex) {
            ex.printStackTrace();
        }
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        servo = s;
        ((CRServoImplEx) servo).setPwmRange(new PwmControl.PwmRange(500, 2500, 5000)); //TODO: figure out what to set framing rate to

        encoder = e;
        rotationController = new PIDFController(-1,-1,-1, 0);
        this.wheel = wheel;

        dynamicTickOffset = getServoEncoderOutput();
    }

    public SwerveModule(@NonNull HardwareMap hardwareMap, String motorName, String servoName, String encoderName, Wheel wheel) {
        this(hardwareMap.get(DcMotorEx.class, motorName), hardwareMap.get(CRServo.class, servoName),
                new AbsoluteAnalogEncoder(hardwareMap.get(AnalogInput.class, encoderName)), wheel);
    }

    protected double getServoEncoderOutput() {
        return encoder.getCurrentPosition();
    }

    public void read() {
        position = getServoEncoderOutput() - dynamicTickOffset;
    }

    public void update(double p, double i, double d) {
        rotationController.setPIDF(p, i, d, 0);
        final double inputTarget = getTargetRotation(), current = getModuleRotation();
        final boolean zeroed = Math.abs(power) <= EPSILON;
        outputTarget =  zeroed ? lastRotationTarget : inputTarget;
        error = normalizeRadians(outputTarget - current);
        if (Math.abs(error) > (FLIP_GAP * Math.PI / 2D)  && !zeroed) {
            outputTarget = normalizeRadians(outputTarget + Math.PI);
            flip = !flip;
            error = normalizeRadians(outputTarget - current);
        }
        final double power = Range.clip(rotationController.calculate(0, error), -MAX_SERVO, MAX_SERVO);
        servo.setPower(Double.isNaN(power) ? 0 : power);
        lastRotationTarget = outputTarget;
        updateMotor();
    }

    public double getTargetRotation() {
        return normalizeRadians((target * POD_GEAR_RATIO) + (flip ? Math.PI : 0));
    }

    public double getModuleRotation() {
        return normalizeRadians(position * POD_GEAR_RATIO);
    }

    protected void setMotorPower(double power) {
        this.power = power;
    }

    private void updateMotor() {
        lastMotorPower = power * MAX_MOTOR * (flip ? -1 : 1);
        motor.setPower(lastMotorPower);
    }

    protected void setTargetRotation(double target) {
        this.target = normalizeRadians(target);
    }

    public void runTelemetry(@NonNull String name, @NonNull Telemetry telemetry) {
        final String caption = "module " + name;
        telemetry.addData(caption + " curr rotation", getModuleRotation());
        telemetry.addData(caption + " raw rotation", position);
        telemetry.addData(caption + " motor power", lastMotorPower);
        telemetry.addData(caption + " raw velocity", motor.getVelocity());
        telemetry.addData(caption + " power from motor", motor.getPower());
        telemetry.addData(caption + " motor velocity", getWheelVelocity());
        telemetry.addData(caption + " motor position", getWheelPosition());
        telemetry.addData(caption + " target rotation", getTargetRotation());
        telemetry.addData(caption + " output target rotation", outputTarget);
        telemetry.addData(caption + " servo power", getServoPower());
        telemetry.addData(caption + " error", error);
    }


    private void setMode(DcMotor.RunMode runMode) {
        motor.setMode(runMode);
    }

    public double getServoPower() {
        return servo.getPower();
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * MOTOR_GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    protected double getWheelPosition() {
        return encoderTicksToInches(motor.getCurrentPosition());
    }

    protected double getWheelVelocity() {
        return encoderTicksToInches(motor.getVelocity());
    }

    public void setTargetModuleState(@NonNull SwerveModuleState state) {
        setMotorPower(state.speedMetersPerSecond);
        setTargetRotation(state.angle.getRadians());
    }
}
