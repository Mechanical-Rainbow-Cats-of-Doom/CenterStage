package org.firstinspires.ftc.teamcode.drive.mecanum;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.drive.localization.Localization2EImpl;

import static java.lang.Math.signum;

@Config
public class AutoMecanum extends MecanumDriveSubsystem {
    private final Localization2EImpl localization;
    public static double defaultPower = 0.3;

    public AutoMecanum(HardwareMap hardwareMap) {
        super(hardwareMap, () -> false, false, null);
        this.localization = new Localization2EImpl(hardwareMap);
    }

    public void initialize() {
        localization.initialize();
    }
    
    public void zero() {
        setTargetVelocity(new ChassisSpeeds());
        periodic();
    }
    
    public void goForward(double inches, double buffer, double power, Runnable runWhileWaiting) {
        final double startingX = localization.getRobotCoordinates().getX();
        setTargetVelocity(new ChassisSpeeds(power, 0, 0));
        periodic();
        while (Math.abs(localization.getPosition().getX() - startingX - inches) >= buffer) {
            runWhileWaiting.run();
        }
        zero();
    }
    
    public void goForward(double inches, double buffer, Runnable runWhileWaiting) {
        goForward(inches, buffer, defaultPower * signum(inches), runWhileWaiting);
    }

    public void goForward(double inches, double buffer, double power) {
        goForward(inches, buffer, power, () -> {});
    }
    
    public void goForward(double inches, double buffer) {
        goForward(inches, buffer, defaultPower * signum(inches));
    }

    public void goSideways(double inches, double buffer, double power, Runnable runWhileWaiting) {
        final double startingY = localization.getRobotCoordinates().getY();
        setTargetVelocity(new ChassisSpeeds(0, power, 0));
        periodic();
        while (Math.abs(localization.getPosition().getY() - startingY - inches) >= buffer) {
            runWhileWaiting.run();
        }
        zero();
    }

    public void goSideways(double inches, double buffer, Runnable runWhileWaiting) {
        goSideways(inches, buffer, defaultPower * signum(inches), runWhileWaiting);
    }

    public void goSideways(double inches, double buffer, double power) {
        goSideways(inches, buffer, power, () -> {});
    }

    public void goSideways(double inches, double buffer) {
        goSideways(inches, buffer, defaultPower * signum(inches));
    }

    public void rotate(double deg, double buffer, double power, Runnable runWhileWaiting) {
        final double startingRotation = localization.getPosition().getHeading();
        setTargetVelocity(new ChassisSpeeds(0, 0, power));
        periodic();
        while (Math.abs(localization.getPosition().getHeading() - startingRotation - deg) >= buffer) {
            runWhileWaiting.run();
        }
        zero();
    }

    public void rotate(double deg, double buffer, Runnable runWhileWaiting) {
        rotate(deg, buffer, defaultPower * signum(deg), runWhileWaiting);
    }

    public void rotate(double deg, double buffer, double power) {
        rotate(deg, buffer, power, () -> {});
    }

    public void rotate(double deg, double buffer) {
        rotate(deg, buffer, defaultPower * signum(deg));
    }

    @Override
    public void periodic() {
        super.periodic();
        localization.updatePosition();
    }
}
