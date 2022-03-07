package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

@Disabled
@TeleOp
public class FSRTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        AnalogInput fsr = hardwareMap.get(AnalogInput.class, "fsr");
        waitForStart();
        while (!isStopRequested()) {
            telemetry.addData("fsr signal", fsr.getVoltage());
            telemetry.update();
        }
    }
}
