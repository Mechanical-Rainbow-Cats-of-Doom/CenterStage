package org.firstinspires.ftc.teamcode.tool;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class PixelSensor extends SubsystemBase {
    public static double ONE_DISTANCE;
    public static double TWO_DISTANCE;

    private final ColorRangeSensor sensor;
    private double distance;
    private int pixelCount;

    public PixelSensor(HardwareMap map) {
        sensor = map.get(ColorRangeSensor.class, "pixelSensor");
    }

    @Override
    public void periodic() {
        distance = sensor.getDistance(DistanceUnit.INCH);
        if(distance > ONE_DISTANCE) {
            pixelCount = 0;
        } else if(distance > TWO_DISTANCE) {
            pixelCount = 1;
        } else {
            pixelCount = 2;
        }

        super.periodic();
    }

    public int getPixelCount() {
        return pixelCount;
    }

    public double getDistance() {
        return distance;
    }
}
