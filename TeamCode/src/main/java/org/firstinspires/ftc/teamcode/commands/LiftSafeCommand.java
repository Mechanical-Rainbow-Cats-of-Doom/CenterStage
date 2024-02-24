//package org.firstinspires.ftc.teamcode.commands;
//
//import com.arcrobotics.ftclib.command.CommandScheduler;
//
//import org.firstinspires.ftc.teamcode.tool.OldIntake;
//import org.firstinspires.ftc.teamcode.tool.old.OldLift;
//
//public class LiftSafeCommand extends LiftGoToPositionCommand {
//    public final PostSafe postSafe;
//    private final OldIntake intake;
//    public LiftSafeCommand(OldIntake intake, OldLift lift) {
//        super(lift, OldLift.LiftPosition.Default.SAFE);
//        this.intake = intake;
//        this.lift = lift;
//        postSafe = new PostSafe(lift);
//    }
//
//    @Override
//    public void end(boolean interrupted) {
//        CommandScheduler.getInstance().schedule(false, postSafe);
//    }
//
//    @Override
//    public boolean isFinished() {
//        return !intake.getOn() || !intake.getState() || !lift.isAutomatic();
//    }
//
//    private static class PostSafe extends LiftGoToPositionCommand {
//        public PostSafe(OldLift lift) {
//            super(lift, OldLift.LiftPosition.Default.DOWN);
//        }
//
//        @Override
//        public boolean isFinished() {
//            return lift.getPosition() != OldLift.LiftPosition.Default.DOWN || lift.getState().isFinished() || !lift.isAutomatic();
//        }
//    }
//}
