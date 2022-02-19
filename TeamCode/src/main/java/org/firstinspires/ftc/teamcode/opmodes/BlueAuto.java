package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.Angle;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Camera;
import org.firstinspires.ftc.teamcode.subsystems.FreightFrenzyRobot;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.OpenCVCamera;

@Config
@Autonomous
public class BlueAuto extends LinearOpMode {
    public static Pose2d INIT_POSE = new Pose2d(-32,64,0);
    public static Pose2d DUCK_SPIN_POSE = new Pose2d(-58, 56, Math.toRadians(90));
    public static double DUCK_SPIN_POWER = 0.25;
    public static double DUCK_SPIN_DURATION = 1.4;
    public static Pose2d DUMP_BLOCK_POSE = new Pose2d(-11, 68, 0);
    public static Pose2d WAREHOUSE_POSE = new Pose2d(44,68,0);

    private NanoClock clock;

    @Override
    public void runOpMode() throws InterruptedException {
        FreightFrenzyRobot robot = new FreightFrenzyRobot(this);
        OpenCVCamera camera = new OpenCVCamera(robot);
        robot.registerSubsystem(camera);
        clock = NanoClock.system();

        robot.drive.setPoseEstimate(INIT_POSE);
        MatchState.CurrentAlliance = MatchState.Alliance.BLUE;
        MatchState.CurrentPosition = MatchState.AutoPosition.DUCK_SPIN;

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

        if (isStopRequested()) return;

        robot.runCommand(robot.drive.followTrajectorySequence(
                robot.drive.trajectorySequenceBuilder(INIT_POSE)
                        .splineToLinearHeading(DUCK_SPIN_POSE, DUCK_SPIN_POSE.getHeading())
                        //.strafeTo(new Vector2d(DUCK_SPIN_POSE.getX() - 0.5, DUCK_SPIN_POSE.getY() + 0.5))
                        .build()
        ));

        robot.runCommand(robot.duck.spinDuck(DUCK_SPIN_POWER, DUCK_SPIN_DURATION));

        robot.runCommand(robot.drive.followTrajectorySequence(
                robot.drive.trajectorySequenceBuilder(DUCK_SPIN_POSE)
                        .turn(-Math.toRadians(90))
                        .addSpatialMarker(new Vector2d(-57, 60), robot.lift::cycleOuttake)
                        .splineToLinearHeading(DUMP_BLOCK_POSE, DUMP_BLOCK_POSE.getHeading())
                        .addSpatialMarker(new Vector2d(-15, 67), robot.lift::cycleOuttake)
                        .build()
        ));

        robot.lift.cycleOuttake();
        robot.update();

        Trajectory warehouseTraj = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                .lineToLinearHeading(WAREHOUSE_POSE)
                .build();

        robot.runCommand(robot.drive.followTrajectorySequence(
                robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                        .waitSeconds(1)
                        .addTrajectory(warehouseTraj)
                        .build()
        ));

        robot.intake.setIntakePower(0.8);

        while (!robot.intake.hasFreight() && !isStopRequested()) {
            if (robot.intake.getIntakeCurrent() < 3.5) {
                robot.drive.setDrivePower(new Pose2d(0.05 +0.1 * Math.sin(4 * clock.seconds()), 0, 0.1 * Math.sin(4 * clock.seconds())));
                robot.intake.setIntakePower(0.8);
            } else {
                robot.drive.setDrivePower(new Pose2d(-0.1,0,0));
                robot.intake.setIntakePower(-0.5);
            }
            robot.update();
        }
        if (isStopRequested()) return;
        robot.drive.setDrivePower(new Pose2d());
        robot.intake.setIntakePower(-0.5);

        robot.lift.setHubLevel(Lift.HubLevel.THIRD);

        Trajectory cycleTraj = robot.drive.trajectoryBuilder(WAREHOUSE_POSE, true)
                .lineToLinearHeading(DUMP_BLOCK_POSE)
                .build();


        robot.runCommand(robot.drive.followTrajectorySequence(
                robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                        .turn(Angle.normDelta(-robot.drive.getPoseEstimate().getHeading()))
                        .splineToLinearHeading(WAREHOUSE_POSE, WAREHOUSE_POSE.getHeading())
                        .addTemporalMarker(() -> {
                            robot.intake.setIntakePower(0);
                            robot.lift.cycleOuttake();
                        })
                        .addTrajectory(cycleTraj)
                        .addSpatialMarker(new Vector2d(-9, 67), robot.lift::cycleOuttake)
                        .waitSeconds(0.25)
                        .build()
        ));

        robot.lift.cycleOuttake();
        robot.update();

        robot.runCommand(robot.drive.followTrajectorySequence(
                robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                        .waitSeconds(1)
                        .addTrajectory(warehouseTraj)
                        .build()
        ));

        while (!robot.intake.hasFreight() && !isStopRequested()) {
            if (robot.intake.getIntakeCurrent() < 3.5) {
                robot.drive.setDrivePower(new Pose2d(0.05 +0.1 * Math.sin(4 * clock.seconds()), 0, 0.1 * Math.sin(4 * clock.seconds())));
                robot.intake.setIntakePower(0.8);
            } else {
                robot.drive.setDrivePower(new Pose2d(-0.1,0,0));
                robot.intake.setIntakePower(-0.5);
            }
            robot.update();
        }
        if (isStopRequested()) return;

        while (!isStopRequested()) {
            robot.update();
        }
    }
}
