package org.firstinspires.ftc.teamcode.test.drive.localization;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;

import org.firstinspires.ftc.teamcode.drive.localization.ContinuousLocalization;
import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class LocalizationTests {
    public static class TestLocalization extends ContinuousLocalization {
        private static class BasicConstantProvider implements ContinuousLocalization.ConstantsProvider {
            public double xMultiplier, yMultiplier;
            public double xOffset, yOffset;

            public BasicConstantProvider(double xMultiplier, double yMultiplier,
                                         double xOffset, double yOffset) {
                this.xMultiplier = xMultiplier;
                this.yMultiplier = yMultiplier;
                this.xOffset = xOffset;
                this.yOffset = yOffset;
            }

            @Override
            public double getXMultiplier() {
                return xMultiplier;
            }

            @Override
            public double getYMultiplier() {
                return yMultiplier;
            }

            @Override
            public double getXOffset() {
                return xOffset;
            }

            @Override
            public double getYOffset() {
                return yOffset;
            }
        }

        public TestLocalization(Pose2d initialPosition, double xMultiplier, double yMultiplier,
                                double xOffset, double yOffset) {
            super(initialPosition, new BasicConstantProvider(xMultiplier, yMultiplier, xOffset, yOffset));
        }


        @Override
        public void updatePosition() {

        }

        public void updatePosition(int xEnc, int yEnc, Rotation2d rotation2d) {
            super.updatePosition(xEnc, yEnc, rotation2d, 0, 0, 0);
        }
    }

    @Test
    public void driveStraightTest() {
        TestLocalization localization = new TestLocalization(
                new Pose2d(new Translation2d(0,0), new Rotation2d(0)),
                1, 1, 0, 0
        );

        localization.updatePosition(0,10,Rotation2d.fromDegrees(0));
        assertEquals(new Pose2d(new Translation2d(0,10), new Rotation2d(0)), localization.getPosition());
    }

    @Test
    public void driveRightTest() {
        TestLocalization localization = new TestLocalization(
                new Pose2d(new Translation2d(0,0), new Rotation2d(0)),
                1, 1, 0, 0
        );

        localization.updatePosition(10,0,Rotation2d.fromDegrees(0));
        assertEquals(new Pose2d(new Translation2d(10,0), new Rotation2d(0)), localization.getPosition());
    }

    @Test
    public void driveAtAngle() {
        TestLocalization localization = new TestLocalization(
                new Pose2d(new Translation2d(0,0), Rotation2d.fromDegrees(45)),
                1, 1, 0, 0
        );

        localization.updatePosition(0,1, Rotation2d.fromDegrees(45));
        assertEquals(new Pose2d(new Translation2d(Math.sqrt(2)/2,Math.sqrt(2)/2), Rotation2d.fromDegrees(45)), localization.getPosition());
    }

    @Test
    public void rotationDriveTest() {
        TestLocalization localization = new TestLocalization(
                new Pose2d(new Translation2d(0,0), Rotation2d.fromDegrees(-90)),
                1, 1, 0, 0
        );

        localization.updatePosition(0,1, Rotation2d.fromDegrees(90));
        assertEquals(new Pose2d(new Translation2d(0,2/Math.PI), Rotation2d.fromDegrees(90)), localization.getPosition());
    }

    @Test
    public void reverseRotationDriveTest() {
        TestLocalization localization = new TestLocalization(
                new Pose2d(new Translation2d(0,0), Rotation2d.fromDegrees(90)),
                1, 1, 0, 0
        );

        localization.updatePosition(0,1, Rotation2d.fromDegrees(-90));
        assertEquals(new Pose2d(new Translation2d(0,2/Math.PI), Rotation2d.fromDegrees(-90)), localization.getPosition());
    }

    @Test
    public void diagonalRotationDriveTest() {
        TestLocalization localization = new TestLocalization(
                new Pose2d(new Translation2d(0,0), Rotation2d.fromDegrees(-45)),
                1, 1, 0, 0
        );

        localization.updatePosition(0,1, Rotation2d.fromDegrees(45));
        assertEquals(new Pose2d(new Translation2d(0, 2/Math.PI*Math.sqrt(2)), Rotation2d.fromDegrees(45)), localization.getPosition());
    }
}
