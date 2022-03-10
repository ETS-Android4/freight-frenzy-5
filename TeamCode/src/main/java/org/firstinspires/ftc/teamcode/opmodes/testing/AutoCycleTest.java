package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.FreightFrenzyRobot;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

@Autonomous
public class AutoCycleTest extends LinearOpMode {
    public static Pose2d WAREHOUSE_POSE = new Pose2d(41,64,-Math.toRadians(5));

    private NanoClock clock;

    @Override
    public void runOpMode() throws InterruptedException {
        FreightFrenzyRobot robot = new FreightFrenzyRobot(this);
        clock = NanoClock.system();

        waitForStart();
        robot.intake.cycleWrist();
        double intakeStartTimestamp = clock.seconds();
        while (!isStopRequested() && !robot.intake.hasFreight() && clock.seconds() - intakeStartTimestamp < 3) {
            robot.update();
            robot.drive.setDrivePower(new Pose2d(0.1 +0.1 * Math.sin(4 * clock.seconds()),0,0));
        }
        robot.drive.setDrivePower(new Pose2d());
        while (!isStopRequested()) {
            robot.update();
        }
    }
}
