package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.framework.Robot;
import org.firstinspires.ftc.teamcode.subsystems.CappingClaw;

@Config
@TeleOp
public class ClawTest extends LinearOpMode {
    public static double armPosition = 0;
    public static double clawPosition = 1;
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this);
        CappingClaw claw = new CappingClaw(robot);
        robot.registerSubsystem(claw);

        waitForStart();

        while (!isStopRequested()) {
            //claw.setArmPosition(armPosition);
            //claw.setClawPosition(clawPosition);
            robot.update();
        }
    }
}
