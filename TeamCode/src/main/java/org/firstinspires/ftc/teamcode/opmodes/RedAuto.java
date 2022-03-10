package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.FreightFrenzyRobot;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

@Config
@Autonomous
public class RedAuto extends LinearOpMode {
    public static Pose2d INIT_POSE = new Pose2d(-32,-64,Math.toRadians(180));
    public static Pose2d TO_DUCK_SPIN = new Pose2d(-48,-40,Math.toRadians(-90));
    public static Pose2d DUCK_SPIN_POSE = new Pose2d(-56, -56, Math.toRadians(-135));
    public static double DUCK_SPIN_POWER = 0.1;
    public static double DUCK_SPIN_DURATION = 4.5;
    public static Pose2d DUMP_BLOCK_POSE = new Pose2d(-52,-24,Math.toRadians(90));
    public static Pose2d PARK_POSE = new Pose2d(-64,-36,Math.toRadians(90));

    private NanoClock clock;

    @Override
    public void runOpMode() throws InterruptedException {
        FreightFrenzyRobot robot = new FreightFrenzyRobot(this);
        //OpenCVCamera camera = new OpenCVCamera(robot);
        //robot.registerSubsystem(camera);
        clock = NanoClock.system();

        robot.drive.setPoseEstimate(INIT_POSE);
        MatchState.CurrentAlliance = MatchState.Alliance.BLUE;
        MatchState.CurrentPosition = MatchState.AutoPosition.DUCK_SPIN;

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

        if (isStopRequested()) return;

        robot.runCommand(robot.drive.followTrajectorySequence(
                robot.drive.trajectorySequenceBuilder(INIT_POSE)
                        .splineToLinearHeading(TO_DUCK_SPIN, TO_DUCK_SPIN.getHeading())
                        .splineToLinearHeading(DUCK_SPIN_POSE, DUCK_SPIN_POSE.getHeading())
                        .build()
        ));

        robot.runCommand(robot.drive.followTrajectory(
                robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                        .splineToLinearHeading(DUCK_SPIN_POSE, DUCK_SPIN_POSE.getHeading())
                        .build()
        ));

        robot.runCommand(robot.duck.spinDuck(DUCK_SPIN_POWER, DUCK_SPIN_DURATION));

        robot.runCommand(robot.drive.followTrajectorySequence(
                robot.drive.trajectorySequenceBuilder(DUCK_SPIN_POSE)
                        .turn(Math.toRadians(-135))
                        .splineToLinearHeading(DUMP_BLOCK_POSE, DUMP_BLOCK_POSE.getHeading())
                        .build()
        ));
        robot.runCommand(robot.drive.followTrajectory(
                robot.drive.trajectoryBuilder(robot.drive.getPoseEstimate())
                        .splineToLinearHeading(DUMP_BLOCK_POSE, DUMP_BLOCK_POSE.getHeading())
                        .build()
        ));
        robot.lift.cycleOuttake();
        while (robot.lift.getOuttakeState() != Lift.OuttakeState.EXTEND && !robot.lift.isLiftExtended()) {
            robot.update();
        }
        robot.runCommand(robot.drive.followTrajectorySequence(
                robot.drive.trajectorySequenceBuilder(DUMP_BLOCK_POSE)
                        .waitSeconds(2)
                        .addTemporalMarker(robot.lift::cycleOuttake)
                        .waitSeconds(0.1)
                        .addTemporalMarker(robot.lift::cycleOuttake)
                        .splineToLinearHeading(PARK_POSE, PARK_POSE.getHeading())
                        .build()
        ));
    }
}
