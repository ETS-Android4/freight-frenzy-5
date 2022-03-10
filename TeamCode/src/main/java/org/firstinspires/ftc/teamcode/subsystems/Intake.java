package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.framework.Robot;
import org.firstinspires.ftc.teamcode.framework.Subsystem;

@Config
public class Intake implements Subsystem {
    private DcMotorEx intakeFront, intakeRear;
    private Servo leftWristFront, rightWristFront, leftWristRear, rightWristRear;
    private Rev2mDistanceSensor distanceSensorFront, distanceSensorRear;
    private NanoClock clock;

    private double intakePower = 0;
    private double wristPosition = WRIST_RETRACT_POSITION;

    public static double WRIST_EXTEND_POSITION = 0.97;
    public static double WRIST_RETRACT_POSITION = 0.1;
    public static double DISTANCE_SENSOR_INTAKE_THRESHOLD = 40;
    public static double DISTANCE_SENSOR_OUTTAKE_THRESHOLD = 100;
    public static double INTAKE_EXTEND_TIME = 1.5;
    public static double INTAKE_STALL_CURRENT = 5;

    private IntakeState state = IntakeState.RETRACT;
    private IntakeDirection direction = IntakeDirection.FRONT;
    private double cachedDistance = 500;
    private double initialTimestamp = 0;

    public enum IntakeState {
        RETRACT,
        INTAKE,
        OUTTAKE
    }

    public enum IntakeDirection {
        FRONT,
        REAR
    }

    public Intake(Robot robot) {
        intakeFront = robot.getMotor("rightEncoder");
        intakeRear = robot.getMotor("leftEncoder");
        intakeFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeFront.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeRear.setDirection(DcMotorSimple.Direction.REVERSE);
        leftWristFront = robot.getServo("intakeLeftFront");
        rightWristFront = robot.getServo("intakeRightFront");
        leftWristRear = robot.getServo("intakeLeftRear");
        rightWristRear = robot.getServo("intakeRightRear");
        leftWristFront.setDirection(Servo.Direction.REVERSE);
        leftWristRear.setDirection(Servo.Direction.REVERSE);
        distanceSensorFront = robot.getHardwareMap().get(Rev2mDistanceSensor.class, "tofFront");
        distanceSensorRear = robot.getHardwareMap().get(Rev2mDistanceSensor.class, "tofRear");
        clock = NanoClock.system();
    }

    public void setIntakePower(double power) {
        intakePower = power;
        if (intakePower > 0) {
            state = IntakeState.INTAKE;
        }
    }

    public boolean hasFreight() {
        return cachedDistance < DISTANCE_SENSOR_INTAKE_THRESHOLD;
    }

    public double getIntakeCurrent() {
        return intakeFront.getCurrent(CurrentUnit.AMPS);
    }

    public IntakeState getIntakeState() {
        return state;
    }

    public void cycleWrist() {
        initialTimestamp = clock.seconds();
        switch (state) {
            case INTAKE:
                state = IntakeState.OUTTAKE;
                break;
            case RETRACT:
            case OUTTAKE:
                state = IntakeState.INTAKE;
                break;
        }
    }

    public void setIntakeDirection(IntakeDirection direction) {
        if (direction != this.direction) {
            state = IntakeState.RETRACT;
        }
        this.direction = direction;
    }

    @Override
    public void update(TelemetryPacket packet) {
        switch (direction) {
            case FRONT:
                cachedDistance = distanceSensorFront.getDistance(DistanceUnit.MM);
                break;
            case REAR:
                cachedDistance = distanceSensorRear.getDistance(DistanceUnit.MM);
                break;
        }
        switch (state) {
            case RETRACT:
                wristPosition = WRIST_RETRACT_POSITION;
                break;
            case INTAKE:
                wristPosition = WRIST_EXTEND_POSITION;
                if (clock.seconds() - initialTimestamp > INTAKE_EXTEND_TIME) {
                    intakePower = 0.7;
                    if (cachedDistance < DISTANCE_SENSOR_INTAKE_THRESHOLD) {
                        state = IntakeState.OUTTAKE;
                        initialTimestamp = clock.seconds();
                    }
                }
                break;
            case OUTTAKE:
                wristPosition = WRIST_RETRACT_POSITION;
                intakePower = -0.1;
                if (clock.seconds() - initialTimestamp > INTAKE_EXTEND_TIME) {
                    intakePower = -1;
                    if (clock.seconds() - initialTimestamp > 1.5 * INTAKE_EXTEND_TIME && cachedDistance > DISTANCE_SENSOR_OUTTAKE_THRESHOLD) {
                        state = IntakeState.RETRACT;
                        intakePower = 0;
                    }
                }
        }
        switch (direction) {
            case FRONT:
                leftWristFront.setPosition(wristPosition);
                rightWristFront.setPosition(wristPosition);
                leftWristRear.setPosition(WRIST_RETRACT_POSITION);
                rightWristRear.setPosition(WRIST_RETRACT_POSITION);
                if (intakeFront.getCurrent(CurrentUnit.AMPS) > INTAKE_STALL_CURRENT) {
                    intakeFront.setPower(-0.1);
                } else {
                    intakeFront.setPower(intakePower);
                }
                if (state == IntakeState.OUTTAKE) {
                    intakeRear.setPower(intakePower);
                } else {
                    intakeRear.setPower(0);
                }
                break;
            case REAR:
                leftWristRear.setPosition(wristPosition);
                rightWristRear.setPosition(wristPosition);
                leftWristFront.setPosition(WRIST_RETRACT_POSITION);
                rightWristFront.setPosition(WRIST_EXTEND_POSITION);
                if (intakeRear.getCurrent(CurrentUnit.AMPS) > INTAKE_STALL_CURRENT) {
                    intakeRear.setPower(-0.1);
                } else {
                    intakeRear.setPower(intakePower);
                }
                if (state == IntakeState.OUTTAKE) {
                    intakeFront.setPower(intakePower);
                } else {
                    intakeFront.setPower(0);
                }
                break;
        }
        packet.put("Intake Current", intakeFront.getCurrent(CurrentUnit.AMPS));
        packet.put("TOF Distance", distanceSensorFront.getDistance(DistanceUnit.MM));
        packet.put("Intake State", state);
    }
}
