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
public class RedAuto2 extends LinearOpMode {
    public static Pose2d INIT_POSE = new Pose2d(9,-64,Math.toRadians(180));
    public static Pose2d FIRST_DUMP_POSE = new Pose2d(-12,-64,Math.toRadians(180));
    public static Pose2d DUMP_BLOCK_POSE = new Pose2d(-9,-64,Math.toRadians(180));
    public static Pose2d WAREHOUSE_POSE = new Pose2d(41,-63,Math.toRadians(180+5));
    public static Pose2d EXIT_WAREHOUSE_POSE = new Pose2d(36,-65,Math.toRadians(180-5));
    public static double intake_move_factor = -1;

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

        // Hardcode hub level
        robot.lift.setHubLevel(Lift.HubLevel.FIRST);

        Trajectory cycleTraj_1 = robot.drive.trajectoryBuilder(EXIT_WAREHOUSE_POSE, true)
                .addSpatialMarker(new Vector2d(24,67), () -> {
                    robot.lift.cycleOuttake();
                    robot.intake.setIntakePower(0);
                })
                .lineToLinearHeading(DUMP_BLOCK_POSE)
                .build();
        Trajectory cycleTraj_2 = robot.drive.trajectoryBuilder(EXIT_WAREHOUSE_POSE, true)
                .addSpatialMarker(new Vector2d(24,67), () -> {
                    robot.lift.cycleOuttake();
                    robot.intake.setIntakePower(0);
                })
                .lineToLinearHeading(DUMP_BLOCK_POSE.plus(new Pose2d(-0.5,0,0)))
                .build();
        Trajectory cycleTraj_3 = robot.drive.trajectoryBuilder(EXIT_WAREHOUSE_POSE, true)
                .addSpatialMarker(new Vector2d(24,67), () -> {
                    robot.lift.cycleOuttake();
                    robot.intake.setIntakePower(0);
                })
                .lineToLinearHeading(DUMP_BLOCK_POSE.plus(new Pose2d(-1,0,0)))
                .build();

        Trajectory traj_backtoWH = robot.drive.trajectoryBuilder(DUMP_BLOCK_POSE, true)
                .lineToLinearHeading(WAREHOUSE_POSE)
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

        // extend outtake
        robot.lift.cycleOuttake(); // extend lift
        // Pre-load freight, go to FIRST_DUMP_POSE
        robot.runCommand(robot.drive.followTrajectorySequence(
                robot.drive.trajectorySequenceBuilder(INIT_POSE)
                        .waitSeconds(1)
                        //.addTemporalMarker(0.1, () -> robot.lift.cycleOuttake()) //extend lift
                        .lineToLinearHeading(FIRST_DUMP_POSE)
                        .waitSeconds(0.5)
                        .build()
        ));

        // Go to warehouse and start intake
        robot.runCommand(robot.drive.followTrajectorySequence(
                robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                        .addTemporalMarker(robot.lift::cycleOuttake) // drop freight
                        .waitSeconds(0.1)
                        .addDisplacementMarker(8, () -> {
                            robot.lift.cycleOuttake(); // fold lift
                            robot.intake.setIntakeDirection(Intake.IntakeDirection.REAR);
                            robot.intake.cycleWrist();
                        })
                        .lineToLinearHeading(WAREHOUSE_POSE) // go back to WH
                        .build()
        ));
        // Intaking movement
        double intakeStartTimestamp = clock.seconds();
        while (    (!isStopRequested())
                && robot.intake.getIntakeState() == Intake.IntakeState.INTAKE
                && (clock.seconds() - intakeStartTimestamp < 3)) {
            robot.update();
            robot.drive.setDrivePower(new Pose2d(intake_move_factor * (0.1 + 0.1 * Math.sin(4 * clock.seconds())),
                    0,
                    intake_move_factor * (-0.05 - 0.05 * Math.sin(4 * clock.seconds()))));
        }
        // stop intaking
        robot.drive.setDrivePower(new Pose2d());
        robot.update();
        if (isStopRequested()) return;
        if (robot.intake.getIntakeState() == Intake.IntakeState.INTAKE) {
            robot.intake.cycleWrist();
        }

        // set hub level to third for repick
        robot.lift.setHubLevel(Lift.HubLevel.THIRD);

        while (clock.seconds() - initialTimestamp < 25) {
            intakeTimes++;
            // move to alliance hub and extend lift
            if (intakeTimes == 1) {
                robot.addCommand(robot.drive.followTrajectorySequence(
                        robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                                .lineToLinearHeading(EXIT_WAREHOUSE_POSE)
                                .addTrajectory(cycleTraj_1)
                                .build()
                ));
            } else if (intakeTimes == 2) {
                robot.addCommand(robot.drive.followTrajectorySequence(
                        robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                                .lineToLinearHeading(EXIT_WAREHOUSE_POSE)
                                .addTrajectory(cycleTraj_2)
                                .build()
                ));
            } else if (intakeTimes == 3) {
                robot.addCommand(robot.drive.followTrajectorySequence(
                        robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                                .lineToLinearHeading(EXIT_WAREHOUSE_POSE)
                                .addTrajectory(cycleTraj_3)
                                .build()
                ));
            } else {
                robot.addCommand(robot.drive.followTrajectorySequence(
                        robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                                .lineToLinearHeading(EXIT_WAREHOUSE_POSE)
                                .addTrajectory(cycleTraj_1)
                                .build()
                ));
            }
            // manually set intake power to 0 after intake state becomes RETRACT
            while (!isStopRequested() && !robot.commandsFinished()) {
                robot.update();
                if (robot.intake.getIntakeState() == Intake.IntakeState.RETRACT) {
                    robot.intake.setIntakePower(0);
                }
            }
            // score freight, then go back to WH
            robot.runCommand(robot.drive.followTrajectorySequence(
                    robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                            .addTemporalMarker(robot.lift::cycleOuttake) // drop freight
                            .waitSeconds(0.1)
                            .addDisplacementMarker(8, () -> {
                                robot.lift.cycleOuttake(); // fold lift
                                robot.intake.cycleWrist();
                            })
                            // go back to warehouse
                            .addTrajectory(traj_backtoWH) // Go back to WH
                            .build()
            ));

            if (isStopRequested()) return;
            // start intaking again
            intakeStartTimestamp = clock.seconds();
            while (!isStopRequested() && robot.intake.getIntakeState() == Intake.IntakeState.INTAKE && clock.seconds() - intakeStartTimestamp < 3) {
                robot.drive.setDrivePower(new Pose2d(intake_move_factor*intakeTimes * (0.1 +0.1 * Math.sin(4 * clock.seconds())),
                        0,
                        intake_move_factor * (-0.05 - 0.05 * Math.sin(4 * clock.seconds()))));
                robot.update();
            }
            // stop intaking
            robot.drive.setDrivePower(new Pose2d());
            robot.update();
            if (isStopRequested()) return;
            if (robot.intake.getIntakeState() == Intake.IntakeState.INTAKE) {
                robot.intake.cycleWrist();
            }
        }
    }
}