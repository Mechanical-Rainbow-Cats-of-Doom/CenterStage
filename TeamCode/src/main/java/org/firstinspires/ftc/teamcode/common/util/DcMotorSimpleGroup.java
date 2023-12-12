package org.firstinspires.ftc.teamcode.common.util;

import android.util.Pair;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.Arrays;
import java.util.List;

import kotlin.NotImplementedError;

public class DcMotorSimpleGroup implements DcMotorSimple {
    private final List<Pair<DcMotorSimple, Boolean>> motors;

    /**
     *
     * @param motors Pair of motor and then inversed boolean
     */
    @SafeVarargs
    public DcMotorSimpleGroup(Pair<DcMotorSimple, Boolean>... motors) {
        this.motors = Arrays.asList(motors);
    }

    @Override
    public void setDirection(Direction direction) {
        throw new NotImplementedError();
    }

    @Override
    public Direction getDirection() {
        throw new NotImplementedError();
    }

    @Override
    public void setPower(double power) {
        motors.forEach(m -> m.first.setPower(power * (m.second ? -1 : 1)));
    }

    @Override
    public double getPower() {
        return motors.get(0).first.getPower();
    }

    @Override
    public Manufacturer getManufacturer() {
        return motors.get(0).first.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return motors.get(0).first.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return motors.get(0).first.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return motors.get(0).first.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        motors.forEach(m -> m.first.resetDeviceConfigurationForOpMode());
    }

    @Override
    public void close() {
        motors.forEach(m -> m.first.close());
    }
}
