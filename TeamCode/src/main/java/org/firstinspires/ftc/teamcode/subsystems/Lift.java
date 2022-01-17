package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.framework.Robot;
import org.firstinspires.ftc.teamcode.framework.Subsystem;

@Config
public class Lift implements Subsystem {
    public static final double PULLEY_RADIUS = 1.0;
    public static final double TICKS_PER_REV = 384.5;
    public static PIDCoefficients LIFT_PID_COEFFICIENTS = new PIDCoefficients(0.8, 0, 0);
    public static double kG = 0.05;
    public static double ARM_MAX_VEL = 4;
    public static double ARM_MAX_ACCEL = 6;

    private DcMotorEx lift;
    private double liftPower = 0;
    private PIDFController liftPidController;
    private NanoClock clock;

    private Servo arm1, arm2;
    private Servo dump;
    private double armPosition = ARM_RETRACT_POSITION;
    private double dumpPosition = DUMPER_RETRACT_POSITION;

    public static double ARM_RETRACT_POSITION = 0.37;
    public static double ARM_LIFT_POSITION = 0.3;
    public static double ARM_THIRD_LEVEL_POSITION = 0.9;
    public static double ARM_SECOND_LEVEL_POSITION = 0.7;
    public static double ARM_FIRST_LEVEL_POSITION = 0.5;
    public static double DUMPER_RETRACT_POSITION = 0.4;
    public static double DUMPER_DUMP_POSITION = 0;
    public static double ARM_EXTEND_TIME = 1;
    private OuttakeState outtakeState = OuttakeState.RETRACT;
    private HubLevel hubLevel = HubLevel.THIRD;
    private double initialTimestamp;
    private boolean armMoving;
    private MotionProfile armMotionProfile;

    public static double LIFT_EXTEND_ARM_POSITION = 9;
    public static double LIFT_DUMP_POSITION = 16;
    public static double LIFT_RETRACT_POSITION = 0;
    public static double LIFT_EXTEND_TIME = 0.5;

    public enum OuttakeState {
        RETRACT,
        EXTEND_ARM,
        EXTEND,
        DUMP,
        RETRACT_ARM
    }

    public enum HubLevel {
        FIRST,
        SECOND,
        THIRD
    }

    public Lift(Robot robot) {
        lift = robot.getMotor("LiftMotor");
        arm1 = robot.getServo("Arm1");
        arm2 = robot.getServo("Arm2");
        dump = robot.getServo("DumpServo");
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        arm2.setDirection(Servo.Direction.REVERSE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftPidController = new PIDFController(LIFT_PID_COEFFICIENTS, 0, 0, kG);
        liftPidController.setOutputBounds(-0.5, 0.5);

        clock = NanoClock.system();
    }

    public static double encoderTicksToInches (int ticks) {
        return PULLEY_RADIUS * 2 * Math.PI * ticks / TICKS_PER_REV;
    }

    public static int encoderInchesToTicks (double inches) {
        return (int) (inches * TICKS_PER_REV / PULLEY_RADIUS / 2 / Math.PI);
    }

    public static double calculateDumperPosition(double armPosition, double dumpPosition) {
        return -armPosition + 1 + dumpPosition;
    }

    public void setLiftPower(double power) {
        liftPower = power;
    }

    public void cycleOuttake() {
        liftPidController.reset();
        switch (outtakeState) {
            case RETRACT:
                outtakeState = OuttakeState.EXTEND_ARM;
                liftPidController.setTargetPosition(LIFT_EXTEND_ARM_POSITION);
                initialTimestamp = clock.seconds();
                break;
            case EXTEND_ARM:
                outtakeState = OuttakeState.EXTEND;
                liftPidController.setTargetPosition(LIFT_DUMP_POSITION);
                break;
            case EXTEND:
                outtakeState = OuttakeState.DUMP;
                break;
            case DUMP:
                outtakeState = OuttakeState.RETRACT_ARM;
                liftPidController.setTargetPosition(LIFT_EXTEND_ARM_POSITION);
                break;
            case RETRACT_ARM:
                outtakeState = OuttakeState.RETRACT;
                liftPidController.setTargetPosition(LIFT_RETRACT_POSITION);
                break;
        }
    }

    public double getArmExtendPosition() {
        switch (hubLevel) {
            case THIRD:
                return ARM_THIRD_LEVEL_POSITION;
            case SECOND:
                return ARM_SECOND_LEVEL_POSITION;
            case FIRST:
                return ARM_FIRST_LEVEL_POSITION;
        }
        return ARM_THIRD_LEVEL_POSITION;
    }

    public void setHubLevel(HubLevel level) {
        hubLevel = level;
    }

    public HubLevel getHubLevel() {
        return hubLevel;
    }

    public double getLiftPosition() {
        return encoderTicksToInches(lift.getCurrentPosition());
    }

    public MotionProfile generateArmMotionProfile(double initialPosition, double endPosition) {
        return MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(initialPosition, 0, 0, 0),
                new MotionState(endPosition, 0, 0, 0),
                ARM_MAX_VEL,
                ARM_MAX_ACCEL
        );
    }

    @Override
    public void update(TelemetryPacket packet) {
        switch (outtakeState) {
            case RETRACT:
                dumpPosition = DUMPER_RETRACT_POSITION;
                if (Math.abs(liftPidController.getLastError()) < 0.2)
                    armPosition = ARM_RETRACT_POSITION;
                else
                    armPosition = ARM_LIFT_POSITION;
                break;
            case EXTEND_ARM:
                dumpPosition = DUMPER_RETRACT_POSITION;
                if (Math.abs(liftPidController.getLastError()) < 0.2 && !armMoving && clock.seconds() - initialTimestamp >= LIFT_EXTEND_TIME) {
                    armMoving = true;
                    initialTimestamp = clock.seconds();
                    armMotionProfile = generateArmMotionProfile(ARM_LIFT_POSITION, getArmExtendPosition());
                }
                else
                    armPosition = ARM_LIFT_POSITION;
                if (armMoving) {
                    if (clock.seconds() - initialTimestamp >= armMotionProfile.duration()) {
                        armMoving = false;
                        outtakeState = OuttakeState.EXTEND;
                        liftPidController.setTargetPosition(LIFT_DUMP_POSITION);
                    } else {
                        armPosition = armMotionProfile.get(clock.seconds() - initialTimestamp).getX();
                    }
                }
                break;
            case EXTEND:
                dumpPosition = DUMPER_RETRACT_POSITION;
                armPosition = getArmExtendPosition();
                break;
            case DUMP:
                dumpPosition = DUMPER_DUMP_POSITION;
                armPosition = getArmExtendPosition();
                break;
            case RETRACT_ARM:
                dumpPosition = DUMPER_RETRACT_POSITION;
                if (Math.abs(liftPidController.getLastError()) < 0.2) {
                    armPosition = ARM_LIFT_POSITION;
                    if (!armMoving) {
                        armMoving = true;
                        initialTimestamp = clock.seconds();
                    }
                }
                else
                    armPosition = getArmExtendPosition();
                if (armMoving && clock.seconds() - initialTimestamp >= ARM_EXTEND_TIME) {
                    armMoving = false;
                    outtakeState = OuttakeState.RETRACT;
                    liftPidController.setTargetPosition(LIFT_RETRACT_POSITION);
                }
                break;
        }
        lift.setPower(liftPidController.update(getLiftPosition()));
        arm1.setPosition(armPosition);
        arm2.setPosition(armPosition);
        dump.setPosition(calculateDumperPosition(armPosition, dumpPosition));
    }
}
