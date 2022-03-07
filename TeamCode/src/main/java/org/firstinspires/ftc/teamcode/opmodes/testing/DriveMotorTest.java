package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous
public class DriveMotorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx driveMotor = hardwareMap.get(DcMotorEx.class, "DriveLF");
        DcMotorEx driveMotor2 = hardwareMap.get(DcMotorEx.class, "DriveLR");
        DcMotorEx driveMotor3 = hardwareMap.get(DcMotorEx.class, "DriveRR");
        DcMotorEx driveMotor4 = hardwareMap.get(DcMotorEx.class, "DriveRF");
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("LF", driveMotor.getCurrentPosition());
            telemetry.addData("LR", driveMotor2.getCurrentPosition());
            telemetry.addData("RR", driveMotor3.getCurrentPosition());
            telemetry.addData("RF", driveMotor4.getCurrentPosition());
            telemetry.update();
        }
    }
}
