package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.framework.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.SimpleMecanumDrive;

@TeleOp
public class MecanumDriveOnly extends LinearOpMode {
    public static double WALL_RUNNER_MULTIPLIER = 0.15;
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this);
        SimpleMecanumDrive mecanumDrive = new SimpleMecanumDrive(robot);
        robot.registerSubsystem(mecanumDrive);
        //DcMotor liftMotor = hardwareMap.get(DcMotor.class, "LiftMotor");
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
                mecanumDrive.setTankPower(-2 * gamepad1.left_stick_y);
                mecanumDrive.retractOdometry();
            } else {
                mecanumDrive.setTankPower(0);
                mecanumDrive.extendOdometry();
            }
            /*
            if (gamepad2.left_bumper) {
                liftMotor.setPower(-1);
            } else {
                liftMotor.setPower(gamepad2.left_trigger);
            }

             */
        }
    }
}
