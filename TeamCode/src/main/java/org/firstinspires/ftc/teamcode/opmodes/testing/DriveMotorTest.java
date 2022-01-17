package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous
public class DriveMotorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx driveMotor = hardwareMap.get(DcMotorEx.class, "DriveRF");
        waitForStart();
        while (opModeIsActive()) {
            driveMotor.setPower(0.25);
            telemetry.addData("Encoder", driveMotor.getCurrentPosition());
            telemetry.update();
        }
    }
}
