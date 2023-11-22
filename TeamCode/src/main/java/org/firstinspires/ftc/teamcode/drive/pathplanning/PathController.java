package org.firstinspires.ftc.teamcode.drive.pathplanning;

import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.HolonomicDrive;
import org.firstinspires.ftc.teamcode.drive.swerve.SwerveDriveSubsystem;

public class PathController <planner extends PathPlanner, holonomicChassis extends HolonomicDrive> {
    public planner pathPlanner;
    public holonomicChassis chassis;

    public PathController(Telemetry telemetry, HardwareMap hmap, planner pathPlanner, holonomicChassis holonomicChassis) {
        this.pathPlanner = pathPlanner;
        this.chassis = holonomicChassis;
    }

    public void run() {
//        ChassisSpeeds
//        chassis.setTargetVelocity();
    }

}
