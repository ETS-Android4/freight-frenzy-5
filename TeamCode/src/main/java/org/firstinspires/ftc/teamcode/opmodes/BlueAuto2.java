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
    public static Pose2d INIT_POSE = new Pose2d(9,64,0);
    public static Pose2d FIRST_DUMP_POSE = new Pose2d(-11,64,0);
    public static Pose2d DUMP_BLOCK_POSE = new Pose2d(-9,64,0);
    public static Pose2d WAREHOUSE_POSE = new Pose2d(41,63,-Math.toRadians(5));
    public static Pose2d EXIT_WAREHOUSE_POSE = new Pose2d(36,65,Math.toRadians(5));

    private NanoClock clock;

    @Override
    public void runOpMode() throws InterruptedException {
        FreightFrenzyRobot robot = new FreightFrenzyRobot(this);
        //OpenCVCamera camera = new OpenCVCamera(robot);
        //robot.registerSubsystem(camera);
        clock = NanoClock.system();
        double intakeTimes = 0;

        robot.drive.setPoseEstimate(INIT_POSE);
        //robot.lift.autoState();
        MatchState.CurrentAlliance = MatchState.Alliance.BLUE;
        MatchState.CurrentPosition = MatchState.AutoPosition.WAREHOUSE;

        robot.lift.setHubLevel(Lift.HubLevel.SECOND);

        Trajectory cycleTraj = robot.drive.trajectoryBuilder(EXIT_WAREHOUSE_POSE, true)
                .addSpatialMarker(new Vector2d(24,67), () -> {
                    robot.lift.cycleOuttake();
                    robot.intake.setIntakePower(0);
                })
                .lineToLinearHeading(DUMP_BLOCK_POSE)
                .build();

        while (!isStarted() && !isStopRequested()) {
            robot.update();
            /*
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

             */
        }
        //camera.shutdown();

        if (isStopRequested()) return;

        double initialTimestamp = clock.seconds();

        robot.lift.cycleOuttake();
        robot.runCommand(robot.drive.followTrajectorySequence(
                robot.drive.trajectorySequenceBuilder(INIT_POSE)
                        .waitSeconds(1)
                        .lineToLinearHeading(FIRST_DUMP_POSE)
                        .waitSeconds(0.1)
                        .build()
        ));

        robot.runCommand(robot.drive.followTrajectorySequence(
                robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                        .addTemporalMarker(robot.lift::cycleOuttake)
                        .waitSeconds(0.1)
                        .addDisplacementMarker(8, () -> {
                            robot.lift.cycleOuttake();
                            robot.intake.cycleWrist();
                        })
                        .lineToLinearHeading(WAREHOUSE_POSE)
                        .build()
        ));
        double intakeStartTimestamp = clock.seconds();
        while (!isStopRequested() && robot.intake.getIntakeState() == Intake.IntakeState.INTAKE && clock.seconds() - intakeStartTimestamp < 3) {
            robot.update();
            robot.drive.setDrivePower(new Pose2d(0.1 +0.1 * Math.sin(4 * clock.seconds()),0,-0.05 - 0.05 * Math.sin(4 * clock.seconds())));
        }
        robot.drive.setDrivePower(new Pose2d());
        robot.update();
        if (isStopRequested()) return;
        if (robot.intake.getIntakeState() == Intake.IntakeState.INTAKE) {
            robot.intake.cycleWrist();
        }
        robot.lift.setHubLevel(Lift.HubLevel.THIRD);

        while (clock.seconds() - initialTimestamp < 25) {
            intakeTimes++;
            robot.addCommand(robot.drive.followTrajectorySequence(
                    robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                            .lineToLinearHeading(EXIT_WAREHOUSE_POSE)
                            .addTrajectory(cycleTraj)
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
                            .addDisplacementMarker(8, () -> {
                                robot.lift.cycleOuttake();
                                robot.intake.cycleWrist();
                            })
                            .splineToLinearHeading(new Pose2d(WAREHOUSE_POSE.getX() + 2 * intakeTimes,
                                    WAREHOUSE_POSE.getY(),
                                    WAREHOUSE_POSE.getHeading()),
                                    WAREHOUSE_POSE.getHeading())
                            .build()
            ));
            if (isStopRequested()) return;
            intakeStartTimestamp = clock.seconds();
            while ( !isStopRequested()
                    && (robot.intake.getIntakeState() == Intake.IntakeState.INTAKE)
                    && (clock.seconds() - intakeStartTimestamp < 3)) {
                robot.drive.setDrivePower(new Pose2d(0.05 +0.05 * Math.sin(4 * clock.seconds()),
                        0,
                        -0.05 - 0.05 * Math.sin(4 * clock.seconds())));
                robot.update();
            }
            robot.drive.setDrivePower(new Pose2d());
            robot.update();
            if (isStopRequested()) return;
            if (robot.intake.getIntakeState() == Intake.IntakeState.INTAKE) {
                robot.intake.cycleWrist();
            }
        }
    }
}
