package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.framework.Robot;
import org.firstinspires.ftc.teamcode.subsystems.SimpleMecanumDrive;

@TeleOp
public class MecanumDriveTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this);
        SimpleMecanumDrive mecanumDrive = new SimpleMecanumDrive(robot);
        robot.registerSubsystem(mecanumDrive);

        waitForStart();

        while (!isStopRequested()) {
            robot.update();
            mecanumDrive.setDrivePower(new Pose2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x));
            if (gamepad1.right_bumper) {
                mecanumDrive.setTankPower(-gamepad1.left_stick_y);
            } else {
                mecanumDrive.setTankPower(0);
            }
        }
    }
}