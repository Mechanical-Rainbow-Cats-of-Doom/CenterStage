package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.tool.Intake;

public abstract class SetIntakeCommands {
    private static abstract class SetIntakeCommand extends CommandBase {
        public final Intake intake;
        public boolean forward;

        public SetIntakeCommand(Intake intake, boolean forward) {
            this.intake = intake;
            this.forward = forward;
        }

        @Override
        public void execute() {
            intake.toggleOn();
            if(!forward) {
                intake.toggleState();
            }
        }

        @Override
        public boolean isFinished() {
            return true;
        }
    }
    public static class SetIntakeForward extends SetIntakeCommand {
        public SetIntakeForward(Intake intake) {
            super(intake, true);
        }
    }
    public static class SetIntakeBackward extends SetIntakeCommand {
        public SetIntakeBackward(Intake intake) {
            super(intake, false);
        }
    }
}
