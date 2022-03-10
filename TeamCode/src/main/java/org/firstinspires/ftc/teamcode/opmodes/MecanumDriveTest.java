package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.commands.core.LynxI2cConfigureChannelCommand;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
    public static double DUCK_SPINNER_MULTIPLIER = 1;
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
        //CappingClaw claw = new CappingClaw(robot);
        //robot.registerSubsystem(claw);
        StickyGamepad stickyGamepad1 = new StickyGamepad(gamepad1);
        robot.addListener(stickyGamepad1);
        StickyGamepad stickyGamepad2 = new StickyGamepad(gamepad2);
        robot.addListener(stickyGamepad2);
        boolean lastHasFreight = false;
        if (MatchState.CurrentAlliance == MatchState.Alliance.RED) {
            DUCK_SPINNER_MULTIPLIER = -DUCK_SPINNER_MULTIPLIER;
            WALL_RUNNER_MULTIPLIER = -WALL_RUNNER_MULTIPLIER;
        }

        waitForStart();

        while (!isStopRequested()) {
            robot.update();
            double xPower = -gamepad1.left_stick_y;
            double yPower = WALL_RUNNER_MULTIPLIER * gamepad1.left_trigger - gamepad1.left_stick_x;
            if (gamepad1.b) {
                yPower *= 2;
            }
            double oPower = -gamepad1.right_stick_x;
            mecanumDrive.setDrivePower(new Pose2d(xPower, yPower, oPower));
            if (gamepad1.left_bumper) {
                mecanumDrive.setTankPower(-2*gamepad1.left_stick_y);
                mecanumDrive.retractOdometry();
            } else {
                mecanumDrive.setTankPower(0);
                mecanumDrive.extendOdometry();
            }
            if (gamepad1.right_bumper) {
                intake.setIntakePower(-1);
            } else {
                double intakePower = 0.8 * gamepad1.right_trigger;
                intake.setIntakePower(intakePower);
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
            if (stickyGamepad2.b) {
                lift.resetLift();
            }
            if (stickyGamepad2.x) {
                lift.incrementArmOffset();
            }
            if (stickyGamepad2.y) {
                lift.decrementArmOffset();
            }
            /*
            if (stickyGamepad2.y) {
                claw.forwardCycle();
            } else if (stickyGamepad2.x) {
                claw.backwardCycle();
            }

             */
            if (stickyGamepad1.y) {
                intake.setIntakeDirection(Intake.IntakeDirection.FRONT);
                intake.cycleWrist();
            } else if (stickyGamepad1.a) {
                intake.setIntakeDirection(Intake.IntakeDirection.REAR);
                intake.cycleWrist();
            }
            if (intake.hasFreight() && !lastHasFreight) {
                gamepad1.rumble(100);
            }
            if (stickyGamepad1.dpad_down) {
                mecanumDrive.toggleWallAlign();
            }
            lastHasFreight = intake.hasFreight();
            if (gamepad2.left_bumper) {
                duckSpinner.setSpinnerPower(0);
            } else if (gamepad2.right_bumper) {
                duckSpinner.setSpinnerPower(1);
            } else {
                duckSpinner.setSpinnerPower(0.5);
            }
            /*
            if (gamepad2.left_bumper) {
                lift.setLiftPower(-1);
            } else {
                lift.setLiftPower(gamepad2.left_trigger);
            }

             */
            telemetry.addData("Current Alliance", MatchState.CurrentAlliance);
            telemetry.addData("Lift position", lift.getLiftPosition());
            telemetry.addData("Intake speed", gamepad1.right_trigger);
            telemetry.update();
        }
    }
}