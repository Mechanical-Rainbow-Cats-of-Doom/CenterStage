package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandScheduler;

import org.firstinspires.ftc.teamcode.tool.Intake;
import org.firstinspires.ftc.teamcode.tool.Lift;

public class LiftSafeCommand extends LiftGoToPositionCommand {
    public final PostSafe postSafe;
    private final Intake intake;
    public LiftSafeCommand(Intake intake, Lift lift) {
        super(lift, Lift.LiftPosition.Default.SAFE);
        this.intake = intake;
        this.lift = lift;
        postSafe = new PostSafe(lift);
    }

    @Override
    public void end(boolean interrupted) {
        CommandScheduler.getInstance().schedule(false, postSafe);
    }

    @Override
    public boolean isFinished() {
        return !intake.getOn() || !intake.getState() || !lift.isAutomatic();
    }

    private static class PostSafe extends LiftGoToPositionCommand {
        public PostSafe(Lift lift) {
            super(lift, Lift.LiftPosition.Default.DOWN);
        }

        @Override
        public boolean isFinished() {
            return lift.getPosition() != Lift.LiftPosition.Default.DOWN || lift.getState().isFinished() || !lift.isAutomatic();
        }
    }
}
