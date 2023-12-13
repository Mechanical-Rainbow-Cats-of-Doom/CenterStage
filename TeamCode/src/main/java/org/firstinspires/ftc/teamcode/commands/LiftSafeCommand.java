package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.tool.Intake;
import org.firstinspires.ftc.teamcode.tool.Lift;

public class LiftSafeCommand extends LiftGoToPositionCommand {
    private final Intake intake;
    public LiftSafeCommand(Intake intake, Lift lift) {
        super(lift, Lift.LiftPosition.Default.SAFE);
        this.intake = intake;
        this.lift = lift;
    }

    @Override
    public void end(boolean interrupted) {
        lift.setPosition(Lift.LiftPosition.Default.DOWN);
    }

    @Override
    public boolean isFinished() {
        return !intake.getOn() || !intake.getState();
    }
}
