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
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.OpenCVCamera;

@Config
@Autonomous
public class Auto2 extends LinearOpMode {
    public static Pose2d INIT_POSE = new Pose2d(9,64,0);
    public static Pose2d DUMP_BLOCK_POSE = new Pose2d(-8, 68, 0);
    public static Pose2d WAREHOUSE_POSE = new Pose2d(48,68,0);

    private NanoClock clock;

    @Override
    public void runOpMode() throws InterruptedException {
        FreightFrenzyRobot robot = new FreightFrenzyRobot(this);
        OpenCVCamera camera = new OpenCVCamera(robot);
        robot.registerSubsystem(camera);
        clock = NanoClock.system();

        robot.drive.setPoseEstimate(INIT_POSE);

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

        double intiialTimestamp = clock.seconds();

        robot.lift.cycleOuttake();

        robot.runCommand(robot.drive.followTrajectorySequence(
                robot.drive.trajectorySequenceBuilder(INIT_POSE)
                        .waitSeconds(1)
                        .lineToLinearHeading(DUMP_BLOCK_POSE)
                        .addTemporalMarker(robot.lift::cycleOuttake)
                        .waitSeconds(0.1)
                        .build()
        ));

        robot.lift.cycleOuttake();

        robot.update();

        Trajectory warehouseTraj = robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                .addSpatialMarker(new Vector2d(33, 64), () -> robot.intake.setIntakePower(0.8))
                .lineToLinearHeading(WAREHOUSE_POSE)
                .build();

        robot.addCommand(robot.drive.followTrajectorySequence(
                robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                        .waitSeconds(0.75)
                        .addTrajectory(warehouseTraj)
                        .build()
        ));

        while (!isStopRequested() && !robot.intake.hasFreight() && !robot.commandsFinished()) {
            robot.update();
        }
        if (isStopRequested()) return;

        robot.clearCommands();

        while (!robot.intake.hasFreight() && !isStopRequested()) {
            if (robot.intake.getIntakeCurrent() < 5) {
                robot.drive.setDrivePower(new Pose2d(0.05 +0.1 * Math.sin(4 * clock.seconds()), 0, 0));
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
                .addSpatialMarker(new Vector2d(-8, 67), robot.lift::cycleOuttake)
                .addTemporalMarker(0.25, robot.lift::cycleOuttake)
                .addTemporalMarker(1, () -> robot.intake.setIntakePower(0))
                .lineToLinearHeading(DUMP_BLOCK_POSE)
                .build();

        while (clock.seconds() - intiialTimestamp < 25){
            robot.runCommand(robot.drive.followTrajectorySequence(
                    robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                            .splineToLinearHeading(WAREHOUSE_POSE, WAREHOUSE_POSE.getHeading())
                            .addTrajectory(cycleTraj)
                            .waitSeconds(0.1)
                            .build()
            ));

            robot.lift.cycleOuttake();
            robot.update();

            robot.addCommand(robot.drive.followTrajectorySequence(
                    robot.drive.trajectorySequenceBuilder(robot.drive.getPoseEstimate())
                            .waitSeconds(0.75)
                            .addTrajectory(warehouseTraj)
                            .build()
            ));

            while (!isStopRequested() && !robot.intake.hasFreight() && !robot.commandsFinished()) {
                robot.update();
            }
            if (isStopRequested()) return;

            robot.clearCommands();

            while (!robot.intake.hasFreight() && !isStopRequested()) {
                if (robot.intake.getIntakeCurrent() < 5) {
                    robot.drive.setDrivePower(new Pose2d(0.05 +0.1 * Math.sin(4 * clock.seconds()), 0, 0));
                    robot.intake.setIntakePower(0.8);
                } else {
                    robot.drive.setDrivePower(new Pose2d(-0.1,0,0));
                    robot.intake.setIntakePower(-0.5);
                }
                robot.update();
            }
            if (isStopRequested()) return;
        }
    }
}
