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
    private DcMotorEx intake;
    private Servo leftWrist, rightWrist;
    private Rev2mDistanceSensor distanceSensor;
    private NanoClock clock;

    private double intakePower = 0;

    public static double WRIST_EXTEND_POSITION = 0.9;
    public static double WRIST_RETRACT_POSITION = 0.01;
    public static double DISTANCE_SENSOR_INTAKE_THRESHOLD = 40;
    public static double DISTANCE_SENSOR_OUTTAKE_THRESHOLD = 100;
    public static double INTAKE_EXTEND_TIME = 1;

    private boolean extended = false;
    private IntakeState state = IntakeState.RETRACT;
    private double cachedDistance = 500;
    private double initialTimestamp = 0;

    public enum IntakeState {
        RETRACT,
        INTAKE,
        OUTTAKE
    }

    public Intake(Robot robot) {
        intake = robot.getMotor("rightEncoder");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        leftWrist = robot.getServo("intakeLeft");
        rightWrist = robot.getServo("intakeRight");
        leftWrist.setDirection(Servo.Direction.REVERSE);
        distanceSensor = robot.getHardwareMap().get(Rev2mDistanceSensor.class, "tof");
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
        return intake.getCurrent(CurrentUnit.AMPS);
    }

    public void cycleWrist() {
        initialTimestamp = clock.seconds();
        switch (state) {
            case INTAKE:
                state = IntakeState.RETRACT;
                break;
            case RETRACT:
            case OUTTAKE:
                state = IntakeState.INTAKE;
                break;
        }
    }

    @Override
    public void update(TelemetryPacket packet) {
        cachedDistance = distanceSensor.getDistance(DistanceUnit.MM);
        switch (state) {
            case RETRACT:
                extended = false;
                break;
            case INTAKE:
                extended = true;
                if (clock.seconds() - initialTimestamp > INTAKE_EXTEND_TIME)
                    intakePower = 0.7;
                if (cachedDistance < DISTANCE_SENSOR_INTAKE_THRESHOLD) {
                    state = IntakeState.OUTTAKE;
                    initialTimestamp = clock.seconds();
                }
                break;
            case OUTTAKE:
                extended = false;
                intakePower = 0.5;
                if (clock.seconds() - initialTimestamp > INTAKE_EXTEND_TIME) {
                    intakePower = -0.7;
                    if (cachedDistance > DISTANCE_SENSOR_OUTTAKE_THRESHOLD) {
                        state = IntakeState.RETRACT;
                    }
                }
        }
        if (intake.getCurrent(CurrentUnit.AMPS) > 4) {
            intakePower *= -0.2;
        }
        if (intakePower > 0 && hasFreight())
            intake.setPower(0);
        else
            intake.setPower(intakePower);
        if (extended) {
            leftWrist.setPosition(WRIST_EXTEND_POSITION);
            rightWrist.setPosition(WRIST_EXTEND_POSITION);
        } else {
            leftWrist.setPosition(WRIST_RETRACT_POSITION);
            rightWrist.setPosition(WRIST_RETRACT_POSITION);
        }
        packet.put("Intake Current", intake.getCurrent(CurrentUnit.AMPS));
        packet.put("TOF Distance", distanceSensor.getDistance(DistanceUnit.MM));
    }
}
