package org.firstinspires.ftc.teamcode.drive;

import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.HolonomicDrive;
import org.firstinspires.ftc.teamcode.drive.localization.Localization2EImpl;
import org.firstinspires.ftc.teamcode.drive.pathplanning.Path;
import org.firstinspires.ftc.teamcode.drive.pathplanning.PathPlanner;

public class DriveController<planner extends PathPlanner, holonomicChassis extends HolonomicDrive & Subsystem> {
    public planner pathPlanner;
    public holonomicChassis chassis;
    // TODO: add a generic for localization, there is 3 wheel, 2 wheel + imu, vision, and more options
    public Localization2EImpl localization;
    public Path path;
    public Telemetry telemetry;

//    public DriveController(HardwareMap hmap, planner pathPlanner, holonomicChassis holonomicChassis, Telemetry t, Path path) {
//        this.pathPlanner = pathPlanner;
//        this.chassis = holonomicChassis;
//        this.localization = new Localization2EImpl(hmap);
//        this.path = path;
//        this.telemetry = t;
//    }

    public DriveController(HardwareMap hmap, planner pathPlanner, holonomicChassis holonomicChassis, Telemetry t) {
        this.pathPlanner = pathPlanner;
        this.chassis = holonomicChassis;
        this.localization = new Localization2EImpl(hmap);
        this.path = new Path();
        this.telemetry = t;
    }

//    public DriveController(HardwareMap hmap, planner pathPlanner, holonomicChassis holonomicChassis) {
//        this.pathPlanner = pathPlanner;
//        this.chassis = holonomicChassis;
//        this.localization = new Localization2EImpl(hmap);
//        this.path = new Path();
//    }

    public void setPath(Path path) {
        this.path = path;
    }

    public Pose2d getPose() {
        return localization.getPosition();
    }

    public boolean isFinished() {
        return this.path.isPathFinished();
    }

    /**
     * Needs to be run as fast as possible in the main loop.
     * Makes the robot move towards the next target point on the path.
     */
    public void run() {
        chassis.periodic();
        localization.updatePosition();
        // gets the optimal velocity between the robot and the target point on the path, increments the target point on the path when reached
        ChassisSpeeds targetVelocity = pathPlanner.calculateTargetVelocity(localization.getVelocity(), localization.getPosition(), path);
        telemetry.addData("R/s: ", targetVelocity.omegaRadiansPerSecond);
        telemetry.addData("vx: ", targetVelocity.vxMetersPerSecond);
        telemetry.addData("vy: ", targetVelocity.vyMetersPerSecond);
        // moves the robot by the optimal velocity. X, Y, THETA
        chassis.setTargetVelocity(targetVelocity);
    }

    /**
     * This is thread blocking, robot does nothing besides drive the full path.
     */
    public void followPath(Path path) {
        setPath(path);
        // moves the robot until it finishes the path
        while(!path.isPathFinished()) {
            chassis.periodic();
            localization.updatePosition();
            // gets the optimal velocity between the robot and the target point on the path, increments the target point on the path when reached
            ChassisSpeeds targetVelocity = pathPlanner.calculateTargetVelocity(localization.getVelocity(), localization.getPosition(), this.path);
            // moves the robot by the optimal velocity. X, Y, THETA
            chassis.setTargetVelocity(targetVelocity);
        }
    }

}
