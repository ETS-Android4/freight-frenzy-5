package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.framework.Robot;
import org.firstinspires.ftc.teamcode.framework.Subsystem;

public class CappingClaw implements Subsystem {
    private Servo armServo;
    private Servo clawServo;
    private double armPosition;
    private double clawPosition;
    private ClawState clawState = ClawState.INIT;

    public static double ARM_DOWN_POSITION = 0;
    public static double ARM_UP_POSITION = 0.45;
    public static double ARM_CAP_POSITION = 0.35;

    public static double CLAW_OPEN_POSITION = 0.7;
    public static double CLAW_CLOSE_POSITION = 0.6;
    public static double CLAW_RETRACT_POSITION = 0.35;

    public enum ClawState {
        INIT,
        GRAB_CAP,
        LIFT_CAP,
        SET_CAP,
        RELEASE_CAP
    }

    public CappingClaw(Robot robot) {
        armServo = robot.getServo("CappingArm");
        clawServo = robot.getServo("CappingClaw");
        armServo.setPosition(0.01);
        clawServo.setPosition(0.35);
    }

    public void forwardCycle() {
        switch (clawState) {
            case INIT:
            case RELEASE_CAP:
                clawState = ClawState.GRAB_CAP;
                break;
            case GRAB_CAP:
                clawState = ClawState.LIFT_CAP;
                break;
            case LIFT_CAP:
                clawState = ClawState.SET_CAP;
                break;
            case SET_CAP:
                clawState = ClawState.RELEASE_CAP;
                break;
        }
    }

    public void backwardCycle() {
        switch (clawState) {
            case RELEASE_CAP:
                clawState = ClawState.SET_CAP;
                break;
            case SET_CAP:
                clawState = ClawState.LIFT_CAP;
                break;
            case LIFT_CAP:
                clawState = ClawState.GRAB_CAP;
                break;
        }
    }

    @Override
    public void update(TelemetryPacket packet) {
        switch (clawState) {
            case INIT:
                armPosition = ARM_DOWN_POSITION;
                clawPosition = CLAW_RETRACT_POSITION;
                break;
            case GRAB_CAP:
                armPosition = ARM_DOWN_POSITION;
                clawPosition = CLAW_OPEN_POSITION;
                break;
            case LIFT_CAP:
                armPosition = ARM_UP_POSITION;
                clawPosition = CLAW_CLOSE_POSITION;
                break;
            case SET_CAP:
                armPosition = ARM_CAP_POSITION;
                clawPosition = CLAW_CLOSE_POSITION;
                break;
            case RELEASE_CAP:
                armPosition = ARM_CAP_POSITION;
                clawPosition = CLAW_OPEN_POSITION;
                break;
        }
        armServo.setPosition(armPosition);
        clawServo.setPosition(clawPosition);

    }
}
