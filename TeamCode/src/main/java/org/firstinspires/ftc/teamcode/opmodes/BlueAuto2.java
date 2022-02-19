package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.Angle;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.FreightFrenzyRobot;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.OpenCVCamera;

@Config
@Autonomous
public class BlueAuto2 extends LinearOpMode {
    public static Pose2d INIT_POSE = new Pose2d(-9,64,0);
    public static Pose2d FIRST_LEVEL_POSE = new Pose2d(-9,50,0);
    public static Pose2d ENTER_WAREHOUSE_POSE = new Pose2d(9,66,0);
    public static Pose2d WAREHOUSE_POSE = new Pose2d(40,66,0);

    private NanoClock clock;

    @Override
    public void runOpMode() throws InterruptedException {
        FreightFrenzyRobot robot = new FreightFrenzyRobot(this);
        OpenCVCamera camera = new OpenCVCamera(robot);
        robot.registerSubsystem(camera);
        clock = NanoClock.system();

        robot.drive.setPoseEstimate(INIT_POSE);
        robot.lift.autoState();
        MatchState.CurrentAlliance = MatchState.Alliance.BLUE;
        MatchState.CurrentPosition = MatchState.AutoPosition.WAREHOUSE;

        while (!isStarted() && !isStopRequested()) {
            robot.update();
            telemetry.addData("Duck Position", camera.getDuckPosition());
            telemetry.addData("Duck X", camera.getDuckX());
            telemetry.addData("Recognition Area", camera.getRecognitionArea());
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

        double initialTimestamp = clock.seconds();

        robot.lift.cycleOuttake();
        while (robot.lift.getOuttakeState() != Lift.OuttakeState.EXTEND && !robot.lift.isLiftExtended()) {
            robot.update();
        }
        robot.runCommand(robot.drive.followTrajectorySequence(
                robot.drive.trajectorySequenceBuilder(INIT_POSE)
                        .waitSeconds(1)
                        .build()
        ));
        if (robot.lift.getHubLevel() == Lift.HubLevel.FIRST) {
            robot.runCommand(robot.drive.followTrajectory(
                    robot.drive.trajectoryBuilder(INIT_POSE)
                            .lineToLinearHeading(FIRST_LEVEL_POSE)
                            .build()
            ));
        }

        robot.runCommand(robot.drive.followTrajectorySequence(
                robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                        .addTemporalMarker(robot.lift::cycleOuttake)
                        .waitSeconds(0.1)
                        .splineToLinearHeading(ENTER_WAREHOUSE_POSE, ENTER_WAREHOUSE_POSE.getHeading())
                        .addTemporalMarker(() -> {
                            robot.lift.cycleOuttake();
                            robot.intake.cycleWrist();
                        })
                        .lineToLinearHeading(WAREHOUSE_POSE)
                        .build()
        ));
        while (!isStopRequested() && robot.intake.getIntakeState() == Intake.IntakeState.INTAKE) {
            robot.drive.setDrivePower(new Pose2d(0.05 +0.1 * Math.sin(4 * clock.seconds()),0,0));
            robot.update();
        }
        if (isStopRequested()) return;
        robot.lift.setHubLevel(Lift.HubLevel.THIRD);

        Trajectory cycleTraj = robot.drive.trajectoryBuilder(WAREHOUSE_POSE, true)
                .addSpatialMarker(new Vector2d(24,67), () -> {
                    robot.lift.cycleOuttake();
                    robot.intake.setIntakePower(0);
                })
                .lineToLinearHeading(INIT_POSE)
                .build();

        while (clock.seconds() - initialTimestamp < 25) {
            robot.addCommand(robot.drive.followTrajectorySequence(
                    robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                            .splineToLinearHeading(WAREHOUSE_POSE, WAREHOUSE_POSE.getHeading())
                            .addTrajectory(cycleTraj)
                            .waitSeconds(0.1)
                            .build()
            ));
            while (!isStopRequested() && !robot.commandsFinished()) {
                robot.update();
                if (robot.intake.getIntakeState() == Intake.IntakeState.RETRACT) {
                    robot.intake.setIntakePower(0);
                }
            }
            robot.runCommand(robot.drive.followTrajectorySequence(
                    robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                            .addTemporalMarker(robot.lift::cycleOuttake)
                            .waitSeconds(0.1)
                            .lineToLinearHeading(ENTER_WAREHOUSE_POSE)
                            .addTemporalMarker(() -> {
                                robot.lift.cycleOuttake();
                                robot.intake.cycleWrist();
                            })
                            .lineToLinearHeading(WAREHOUSE_POSE)
                            .build()
            ));
            if (isStopRequested()) return;
            while (!isStopRequested() && robot.intake.getIntakeState() == Intake.IntakeState.INTAKE) {
                robot.drive.setDrivePower(new Pose2d(0.05 +0.1 * Math.sin(4 * clock.seconds()),0,0));
                robot.update();
            }
            if (isStopRequested()) return;
        }
    }
}
