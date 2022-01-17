package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.framework.Robot;
import org.firstinspires.ftc.teamcode.hardware.StickyGamepad;
import org.firstinspires.ftc.teamcode.subsystems.DuckSpinner;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.SimpleMecanumDrive;

@Config
@TeleOp
public class MecanumDriveTest extends LinearOpMode {
    public static double WALL_RUNNER_MULTIPLIER = 0.15;
    public static double DUCK_SPINNER_MULTIPLIER = 0.25;
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this);
        SimpleMecanumDrive mecanumDrive = new SimpleMecanumDrive(robot);
        robot.registerSubsystem(mecanumDrive);
        DuckSpinner duckSpinner = new DuckSpinner(robot);
        robot.registerSubsystem(duckSpinner);
        Intake intake = new Intake(robot);
        robot.registerSubsystem(intake);
        Lift lift = new Lift(robot);
        robot.registerSubsystem(lift);
        StickyGamepad stickyGamepad1 = new StickyGamepad(gamepad1);
        robot.addListener(stickyGamepad1);
        StickyGamepad stickyGamepad2 = new StickyGamepad(gamepad2);
        robot.addListener(stickyGamepad2);

        waitForStart();

        while (!isStopRequested()) {
            robot.update();
            double xPower = -gamepad1.left_stick_y;
            double yPower = WALL_RUNNER_MULTIPLIER * gamepad1.right_trigger - gamepad1.left_stick_x;
            if (gamepad1.b) {
                yPower *= 2;
            }
            double oPower = -gamepad1.right_stick_x;
            mecanumDrive.setDrivePower(new Pose2d(xPower, yPower, oPower));
            if (gamepad1.right_bumper) {
                mecanumDrive.setTankPower(-gamepad1.left_stick_y);
                mecanumDrive.retractOdometry();
            } else {
                mecanumDrive.setTankPower(0);
                mecanumDrive.extendOdometry();
            }
            if (gamepad1.left_bumper) {
                intake.setIntakePower(-1);
            } else {
                intake.setIntakePower(gamepad1.left_trigger);
            }
            if (stickyGamepad2.a) {
                lift.cycleOuttake();
            }
            if (gamepad2.dpad_up) {
                lift.setHubLevel(Lift.HubLevel.THIRD);
            } else if (gamepad2.dpad_left) {
                lift.setHubLevel(Lift.HubLevel.SECOND);
            } else  if (gamepad2.dpad_down) {
                lift.setHubLevel(Lift.HubLevel.FIRST);
            }
            duckSpinner.setSpinnerPower(DUCK_SPINNER_MULTIPLIER*gamepad2.right_trigger);
            telemetry.addData("Lift position", lift.getLiftPosition());
            telemetry.addData("Intake speed", gamepad1.left_trigger);
            telemetry.update();
        }
    }
}