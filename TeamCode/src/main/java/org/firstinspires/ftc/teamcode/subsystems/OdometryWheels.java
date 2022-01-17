package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;

import org.firstinspires.ftc.teamcode.framework.Robot;
import org.firstinspires.ftc.teamcode.hardware.Encoder;

import java.util.Arrays;
import java.util.List;

@Config
public class OdometryWheels extends ThreeTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 0.6889765; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 10.404; // in; distance between the left and right wheels
    public static double FORWARD_OFFSET = -7.53; // in; offset of the lateral wheel

    private Encoder leftEncoder, rightEncoder, frontEncoder;

    public OdometryWheels(Robot robot) {
        super(Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(-90)) // front
        ));

        leftEncoder = robot.getEncoder("leftEncoder");
        rightEncoder = robot.getEncoder("rightEncoder");
        frontEncoder = robot.getEncoder("frontEncoder");
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCurrentPosition()),
                encoderTicksToInches(rightEncoder.getCurrentPosition()),
                encoderTicksToInches(frontEncoder.getCurrentPosition())
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCorrectedVelocity()),
                encoderTicksToInches(rightEncoder.getCorrectedVelocity()),
                encoderTicksToInches(frontEncoder.getCorrectedVelocity())
        );
    }
}
