package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.framework.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

@Config
@Autonomous
public class AutoPathing extends LinearOpMode {
    public static Pose2d INIT_POSE = new Pose2d(-32,64,0);
    public static Pose2d DUCK_SPIN_POSE = new Pose2d(-58, 56, Math.toRadians(90));
    public static double DUCK_SPIN_POWER = 0.3;
    public static double DUCK_SPIN_DURATION = 1.4;
    public static Pose2d DUMP_BLOCK_POSE = new Pose2d(-9, 67, 0);
    public static Pose2d FIRST_DUMP_POSE = new Pose2d(-10, 54, 0);
    public static Pose2d CROSS_BARRIER_POSE = new Pose2d(24, 71, 0);
    public static Pose2d WAREHOUSE_POSE = new Pose2d(48,67,0);

    private Pose2d dumpPose = DUMP_BLOCK_POSE;
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this);
        Drivetrain drivetrain = new Drivetrain(robot);
        robot.registerSubsystem(drivetrain);

        drivetrain.setPoseEstimate(INIT_POSE);

        waitForStart();

        robot.runCommand(drivetrain.followTrajectory(
                drivetrain.trajectoryBuilder(INIT_POSE)
                        .splineToLinearHeading(DUCK_SPIN_POSE, DUCK_SPIN_POSE.getHeading())
                        .build()
        ));

        robot.runCommand(drivetrain.followTrajectorySequence(
                drivetrain.trajectorySequenceBuilder(DUCK_SPIN_POSE)
                        .waitSeconds(DUCK_SPIN_DURATION)
                        .turn(-Math.toRadians(90))
                        .splineToLinearHeading(dumpPose, dumpPose.getHeading())
                        .waitSeconds(0.25)
                        .build()
        ));

        robot.runCommand(drivetrain.followTrajectorySequence(
                drivetrain.trajectorySequenceBuilder(drivetrain.getPoseEstimate())
                        .waitSeconds(0.5)
                        .lineToLinearHeading(WAREHOUSE_POSE)
                        .build()
        ));

        robot.runCommand(drivetrain.followTrajectorySequence(
                drivetrain.trajectorySequenceBuilder(WAREHOUSE_POSE)
                        .waitSeconds(0.5)
                        .lineToLinearHeading(dumpPose)
                        .build()
        ));

        robot.runCommand(drivetrain.followTrajectorySequence(
                drivetrain.trajectorySequenceBuilder(dumpPose)
                        .waitSeconds(0.5)
                        .lineToLinearHeading(WAREHOUSE_POSE)
                        .build()
        ));

        while (!isStopRequested()) {
            robot.update();
        }
    }
}
