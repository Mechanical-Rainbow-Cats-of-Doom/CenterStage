package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.tool.Intake;
import org.firstinspires.ftc.teamcode.tool.old.OldLift;

public class LiftSafeCommand extends LiftGoToPositionCommand {
    private final Intake intake;
    public LiftSafeCommand(Intake intake, OldLift lift) {
        super(lift, OldLift.LiftPosition.Default.SAFE);
        this.intake = intake;
        this.lift = lift;
    }

    @Override
    public void end(boolean interrupted) {
        lift.setPosition(OldLift.LiftPosition.Default.DOWN);
    }

    @Override
    public boolean isFinished() {
        return !intake.getOn() || !intake.getState();
    }
}
