package org.firstinspires.ftc.teamcode.tool;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class LightingSubsystem extends SubsystemBase {
    public static RevBlinkinLedDriver.BlinkinPattern RED_ALLIANCE_COLOR = RevBlinkinLedDriver.BlinkinPattern.RED;
    public static RevBlinkinLedDriver.BlinkinPattern BLUE_ALLIANCE_COLOR = RevBlinkinLedDriver.BlinkinPattern.BLUE;
    public static RevBlinkinLedDriver.BlinkinPattern TOUCHING_BOARD_COLOR = RevBlinkinLedDriver.BlinkinPattern.GREEN;
    public static RevBlinkinLedDriver.BlinkinPattern ONE_PIXEL_COLOR = RevBlinkinLedDriver.BlinkinPattern.GOLD;
    public static RevBlinkinLedDriver.BlinkinPattern TWO_PIXEL_COLOR = RevBlinkinLedDriver.BlinkinPattern.GREEN;

    private final boolean isRed;
    private final RevBlinkinLedDriver boardSensorBlinkin;
    private final RevBlinkinLedDriver pixelDistanceBlinkin;
    private final PixelSensor pixelSensor;
    private final TouchSensor boardSensor;
    private final Telemetry telemetry;

    public LightingSubsystem(HardwareMap hardwareMap, boolean isRed, Telemetry telemetry) {
        pixelSensor = new PixelSensor(hardwareMap);
        boardSensorBlinkin = hardwareMap.get(RevBlinkinLedDriver.class, "frontBlinkin");
        pixelDistanceBlinkin = hardwareMap.get(RevBlinkinLedDriver.class, "backBlinkin");
        boardSensor = hardwareMap.get(TouchSensor.class, "boardSensor");
        this.isRed = isRed;
        this.telemetry = telemetry;
    }

    public LightingSubsystem(HardwareMap hardwareMap, boolean isRed) {
        this(hardwareMap, isRed, null);
    }

    @Override
    public void periodic() {
        pixelSensor.periodic();
        boolean touchingBoard = boardSensor.isPressed();
        if(telemetry != null) {
            telemetry.addData("Pixel Sensor Distance", pixelSensor.getDistance());
            telemetry.addData("Is Touching Board", touchingBoard);
        }

        switch(pixelSensor.getPixelCount()) {
            default:
            case 0:
                pixelDistanceBlinkin.setPattern(getTeamColor());
                break;
            case 1:
                pixelDistanceBlinkin.setPattern(ONE_PIXEL_COLOR);
                break;
            case 2:
                pixelDistanceBlinkin.setPattern(TWO_PIXEL_COLOR);
                break;
        }
        boardSensorBlinkin.setPattern(touchingBoard ? TOUCHING_BOARD_COLOR : getTeamColor());
    }

    private RevBlinkinLedDriver.BlinkinPattern getTeamColor() {
        return isRed ? RED_ALLIANCE_COLOR : BLUE_ALLIANCE_COLOR;
    }
}
