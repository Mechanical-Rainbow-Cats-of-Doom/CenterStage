package org.firstinspires.ftc.teamcode.drive.pathplanning;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;

public interface PathPlanner {
    // this method is probably very wrong please feel free to change it because it doesnt take into account states in trajectory and stuff
    // please keep in mind that the teleop classes will not be directly interfacing with the path planner, it will be through the robot by setting trajectories for it
    // please make sure this design paradigm works with whatever class you write.
    ChassisSpeeds calculateTargetVelocity(ChassisSpeeds currentVelocity, Pose2d currentPose, Path path);

    /**
     * Used for making sure path addition methods interface properly
     * NOT IMPLEMENTED AS OF WRITING
     */
    String getName();
}
