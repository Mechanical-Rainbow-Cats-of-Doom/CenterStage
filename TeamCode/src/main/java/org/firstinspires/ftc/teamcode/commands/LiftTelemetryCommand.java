package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.tool.old.OldLift;

public class LiftTelemetryCommand extends CommandBase {
    private final Telemetry telemetry;
    private final OldLift lift;

    public LiftTelemetryCommand(Telemetry telemetry, OldLift lift) {
        this.telemetry = telemetry;
        this.lift = lift;
    }

    @Override
    public void execute() {
        telemetry.addData("Current State", lift.getState().toString());
        telemetry.addData("Current Position", lift.getPosition().toString());
        telemetry.addData("Current Error", lift.getError());
        telemetry.addData("Current Power", lift.getPower());
        telemetry.addData("Claw Open", lift.getClawOpen());
    }
}
