package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.FreightFrenzyRobot;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

@Autonomous
public class AutoCycleTest extends LinearOpMode {
    public static Pose2d DUMP_BLOCK_POSE = new Pose2d(-10, 67 - 0.375, 0);
    public static Pose2d FIRST_DUMP_POSE = new Pose2d(-10, 54, 0);
    public static Pose2d CROSS_BARRIER_POSE = new Pose2d(24, 71, 0);
    public static Pose2d WAREHOUSE_POSE = new Pose2d(48,71,0);

    private Pose2d dumpPose = DUMP_BLOCK_POSE;

    private NanoClock clock;

    @Override
    public void runOpMode() throws InterruptedException {
        FreightFrenzyRobot robot = new FreightFrenzyRobot(this);
        clock = NanoClock.system();

        robot.drive.setPoseEstimate(WAREHOUSE_POSE);

        waitForStart();
        if (isStopRequested()) return;

        robot.intake.setIntakePower(1);

        while (!robot.intake.hasFreight() && !isStopRequested()) {
            robot.drive.setDrivePower(new Pose2d(0.1 + 0.1 * Math.sin(clock.seconds()),0,0.1 * Math.sin(clock.seconds())));
            robot.update();
        }
        if (isStopRequested()) return;

        robot.drive.setDrivePower(new Pose2d());
        robot.intake.setIntakePower(-0.5);
        while (!isStopRequested()) {
            robot.update();
        }
        /*
        robot.lift.setHubLevel(Lift.HubLevel.THIRD);
        dumpPose = DUMP_BLOCK_POSE;

        Trajectory cycleTraj = robot.drive.trajectoryBuilder(WAREHOUSE_POSE, true)
                .splineToSplineHeading(CROSS_BARRIER_POSE, CROSS_BARRIER_POSE.getHeading())
                .addSpatialMarker(new Vector2d(12, 70), () -> {
                    robot.intake.setIntakePower(0);
                    robot.lift.cycleOuttake();
                })
                .splineToSplineHeading(DUMP_BLOCK_POSE, DUMP_BLOCK_POSE.getHeading())
                .build();


        robot.runCommand(robot.drive.followTrajectorySequence(
                robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                        .splineToLinearHeading(WAREHOUSE_POSE, WAREHOUSE_POSE.getHeading())
                        .addTrajectory(cycleTraj)
                        .addDisplacementMarker(robot.lift::cycleOuttake)
                        .waitSeconds(0.25)
                        .build()
        ));

        robot.lift.cycleOuttake();

        Trajectory warehouseTraj = robot.drive.trajectoryBuilder(dumpPose)
                .splineToSplineHeading(CROSS_BARRIER_POSE, CROSS_BARRIER_POSE.getHeading())
                .splineToSplineHeading(WAREHOUSE_POSE, WAREHOUSE_POSE.getHeading())
                .build();

        robot.runCommand(robot.drive.followTrajectory(
                warehouseTraj
        ));

        while (!isStopRequested()) {
            robot.update();
        }
         */
    }
}
