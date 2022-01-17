package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Camera;
import org.firstinspires.ftc.teamcode.subsystems.FreightFrenzyRobot;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

@Config
@Autonomous
public class Auto  extends LinearOpMode {
    public static Pose2d INIT_POSE = new Pose2d(-32,64,0);
    public static Pose2d DUCK_SPIN_POSE = new Pose2d(-58, 56, Math.toRadians(90));
    public static double DUCK_SPIN_POWER = 0.3;
    public static double DUCK_SPIN_DURATION = 1.4;
    public static Pose2d DUMP_BLOCK_POSE = new Pose2d(-10, 67 - 0.375, 0);
    public static Pose2d FIRST_DUMP_POSE = new Pose2d(-10, 54, 0);
    public static Pose2d CROSS_BARRIER_POSE = new Pose2d(24, 71, 0);
    public static Pose2d WAREHOUSE_POSE = new Pose2d(48,71,0);

    private Pose2d dumpPose = DUMP_BLOCK_POSE;

    private NanoClock clock;

    @Override
    public void runOpMode() throws InterruptedException {
        FreightFrenzyRobot robot = new FreightFrenzyRobot(this);
        Camera camera = new Camera(robot);
        robot.registerSubsystem(camera);
        clock = NanoClock.system();

        robot.drive.setPoseEstimate(INIT_POSE);

        while (!isStarted() && !isStopRequested()) {
            robot.update();
            telemetry.addData("Duck X", camera.getDuckX());
            telemetry.addData("Duck Y", camera.getDuckY());
            telemetry.addData("Duck Position", camera.getDuckPosition());
            telemetry.update();
            switch (camera.getDuckPosition()) {
                case LEFT:
                    robot.lift.setHubLevel(Lift.HubLevel.FIRST);
                    break;
                case MIDDLE:
                    robot.lift.setHubLevel(Lift.HubLevel.SECOND);
                    break;
                case RIGHT:
                    robot.lift.setHubLevel(Lift.HubLevel.THIRD);
            }
        }
        camera.shutdown();
        if (isStopRequested()) return;

        robot.runCommand(robot.drive.followTrajectorySequence(
                robot.drive.trajectorySequenceBuilder(INIT_POSE)
                        .splineToLinearHeading(DUCK_SPIN_POSE, DUCK_SPIN_POSE.getHeading())
                        //.strafeTo(new Vector2d(DUCK_SPIN_POSE.getX() - 0.5, DUCK_SPIN_POSE.getY() + 0.5))
                        .build()
        ));

        robot.runCommand(robot.duck.spinDuck(DUCK_SPIN_POWER, DUCK_SPIN_DURATION));

        if (robot.lift.getHubLevel() == Lift.HubLevel.FIRST) {
            dumpPose = FIRST_DUMP_POSE;
        }

        robot.runCommand(robot.drive.followTrajectorySequence(
                robot.drive.trajectorySequenceBuilder(DUCK_SPIN_POSE)
                        .turn(-Math.toRadians(90))
                        .addSpatialMarker(new Vector2d(-54, 60), robot.lift::cycleOuttake)
                        .splineToLinearHeading(dumpPose, dumpPose.getHeading())
                        .addTemporalMarker(robot.lift::cycleOuttake)
                        .waitSeconds(0.25)
                        .build()
        ));

        robot.lift.cycleOuttake();
        robot.update();

        Trajectory warehouseTraj = robot.drive.trajectoryBuilder(dumpPose)
                .splineToSplineHeading(CROSS_BARRIER_POSE, CROSS_BARRIER_POSE.getHeading())
                .splineToSplineHeading(WAREHOUSE_POSE, WAREHOUSE_POSE.getHeading())
                .build();

        robot.runCommand(robot.drive.followTrajectorySequence(
                robot.drive.trajectorySequenceBuilder(DUMP_BLOCK_POSE)
                        .waitSeconds(0.5)
                        .addTrajectory(warehouseTraj)
                        .build()
        ));

        robot.intake.setIntakePower(1);

        while (!robot.intake.hasFreight() && !isStopRequested()) {
            robot.drive.setDrivePower(new Pose2d(0.1 + 0.1 * Math.sin(clock.seconds()),0,0.1 * Math.sin(clock.seconds())));
            robot.update();
        }
        if (isStopRequested()) return;
        robot.drive.setDrivePower(new Pose2d());
        robot.intake.setIntakePower(-0.5);

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

        robot.runCommand(robot.drive.followTrajectory(
                warehouseTraj
        ));

        while (!isStopRequested()) {
            robot.update();
        }
    }
}
