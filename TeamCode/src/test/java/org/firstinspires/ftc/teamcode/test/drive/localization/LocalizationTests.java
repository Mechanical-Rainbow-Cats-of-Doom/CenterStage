package org.firstinspires.ftc.teamcode.test.drive.localization;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

import org.firstinspires.ftc.teamcode.drive.localization.Localization2EImpl;
import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class LocalizationTests {
    @Test
    public void driveStraightTest() {
        Localization2EImpl localization = new Localization2EImpl(
                new Pose2d(new Translation2d(0,0), new Rotation2d(0)),
                1, 1, 0, 0
        );

        localization.updatePosition(0,10,0);
        assertEquals(new Pose2d(new Translation2d(0,10), new Rotation2d(0)), localization.getPosition());
    }

    @Test
    public void driveRightTest() {
        Localization2EImpl localization = new Localization2EImpl(
                new Pose2d(new Translation2d(0,0), new Rotation2d(0)),
                1, 1, 0, 0
        );

        localization.updatePosition(10,0,0);
        assertEquals(new Pose2d(new Translation2d(10,0), new Rotation2d(0)), localization.getPosition());
    }

    @Test
    public void driveAtAngle() {
        Localization2EImpl localization = new Localization2EImpl(
                new Pose2d(new Translation2d(0,0), Rotation2d.fromDegrees(45)),
                1, 1, 0, 0
        );

        localization.updatePosition(0,1, Math.toRadians(45));
        assertEquals(new Pose2d(new Translation2d(Math.sqrt(2)/2,Math.sqrt(2)/2), Rotation2d.fromDegrees(45)), localization.getPosition());
    }

    @Test
    public void rotationDriveTest() {
        Localization2EImpl localization = new Localization2EImpl(
                new Pose2d(new Translation2d(0,0), Rotation2d.fromDegrees(-45)),
                1, 1, 0, 0
        );

        localization.updatePosition(0,1, Math.toRadians(45));
        assertEquals(new Pose2d(new Translation2d(0,1), Rotation2d.fromDegrees(45)), localization.getPosition());
    }
}
