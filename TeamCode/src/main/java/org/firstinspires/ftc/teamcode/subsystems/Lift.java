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
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.framework.Robot;
import org.firstinspires.ftc.teamcode.framework.Subsystem;

@Config
public class Lift implements Subsystem {
    public static final double PULLEY_RADIUS = 0.626496;
    public static final double TICKS_PER_REV = 101.08;
    public static PIDCoefficients LIFT_PID_COEFFICIENTS = new PIDCoefficients(0.2, 0.05, 0.01);
    public static double kG = 0.0;
    public static double ARM_MAX_VEL = 4;
    public static double ARM_MAX_ACCEL = 4;

    private DcMotorEx lift;
    private double liftPower = 0;
    private PIDFController liftPidController;
    private NanoClock clock;

    private Servo arm1, arm2;
    private Servo dump;
    private Servo locker;
    private double armPosition = ARM_RETRACT_POSITION;
    private double dumpPosition = DUMPER_RETRACT_POSITION;
    private double lockerPosition = LOCKER_UNLOCK_POSITION;
    private double armOffset = 0;

    private TouchSensor limitSwitch;

    public static double ARM_RETRACT_POSITION = 0.03;
    public static double ARM_PICK_FREIGHT_POSITION = 0.01;
    public static double ARM_LIFT_POSITION = 0.4;
    public static double ARM_THIRD_LEVEL_POSITION = 0.7;
    public static double ARM_SECOND_LEVEL_POSITION = 0.85;
    public static double ARM_FIRST_LEVEL_POSITION = 0.95;
    public static double ARM_SHARED_HUB_POSITION = 0.9;
    public static double DUMPER_RETRACT_POSITION = 0.685;
    public static double DUMPER_LIFTING_POSITION = 0.75;
    public static double DUMPER_AUTO_POSITION = 0.4;
    public static double DUMPER_DUMP_POSITION = 0.2;
    //public static double DUMPER_FIRST_LEVEL_POSITION = 0.23;
    public static double DUMPER_SHARED_HUB_POSITION = 0.15;
    public static double LOCKER_LOCK_POSITION = 1;
    public static double LOCKER_UNLOCK_POSITION = 0.5;
    public static double LOCKER_HOLDING_POSITION = 0.77;
    public static double ARM_EXTEND_TIME = 0.7;
    public static double ARM_OFFSET = 0;
    private OuttakeState outtakeState = OuttakeState.RETRACT;
    private HubLevel hubLevel = HubLevel.THIRD;
    private LiftMode liftMode = LiftMode.PID;
    private double initialTimestamp;
    private boolean armMoving;
    private MotionProfile armMotionProfile;

    public static double LIFT_DUMP_POSITION = 20;
    public static double LIFT_SECOND_LEVEL_POSITION = 18;
    public static double LIFT_FIRST_LEVEL_POSITION = 16;
    public static double LIFT_SHARED_HUB_POSITION = 4;
    public static double LIFT_RETRACT_POSITION = 0;
    public static double LIFT_EXTEND_TIME = 1.5;

    public enum OuttakeState {
        AUTO,
        AUTO_EXTEND,
        RETRACT,
        PICK_FREIGHT,
        EXTEND_ARM,
        EXTEND,
        DUMP,
        RETRACT_ARM
    }

    public enum HubLevel {
        FIRST,
        SECOND,
        THIRD,
        SHARED
    }

    public enum LiftMode {
        MANUAL,
        PID,
        RESET
    }

    public Lift(Robot robot) {
        lift = robot.getMotor("LiftMotor");
        arm1 = robot.getServo("Arm1");
        arm2 = robot.getServo("Arm2");
        dump = robot.getServo("DumpServo");
        locker = robot.getServo("Locker");
        //lift.setDirection(DcMotorSimple.Direction.REVERSE);
        arm1.setDirection(Servo.Direction.REVERSE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftPidController = new PIDFController(LIFT_PID_COEFFICIENTS, 0, 0, kG);
        liftPidController.setOutputBounds(-0.7, 0.8);

        clock = NanoClock.system();
        limitSwitch = robot.getHardwareMap().get(TouchSensor.class, "limitSwitch");
    }

    public static double encoderTicksToInches (int ticks) {
        return PULLEY_RADIUS * 2 * Math.PI * ticks / TICKS_PER_REV;
    }

    //never used lol
    public static int encoderInchesToTicks (double inches) {
        return (int) (inches * TICKS_PER_REV / PULLEY_RADIUS / 2 / Math.PI);
    }

    //deprecated
    public static double calculateDumperPosition(double armPosition, double dumpPosition) {
        return -armPosition + 1 + dumpPosition;
    }

    public void setLiftPower(double power) {
        liftPower = power;
    }

    public void cycleOuttake() {
        liftMode = LiftMode.PID;
        liftPidController.reset();
        switch (outtakeState) {
            case AUTO:
                outtakeState = OuttakeState.AUTO_EXTEND;
                initialTimestamp = clock.seconds();
                break;
            case RETRACT:
                outtakeState = OuttakeState.EXTEND_ARM;
                //liftPidController.setTargetPosition(LIFT_DUMP_POSITION);
                initialTimestamp = clock.seconds();
                break;
            case EXTEND_ARM:
                outtakeState = OuttakeState.EXTEND;
                liftPidController.setTargetPosition(getLiftExtendPosition());
                break;
            case EXTEND:
                outtakeState = OuttakeState.DUMP;
                break;
            case DUMP:
                outtakeState = OuttakeState.RETRACT_ARM;
                liftPidController.setTargetPosition(LIFT_RETRACT_POSITION);
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
            case SHARED:
                return ARM_SHARED_HUB_POSITION;
        }
        return ARM_THIRD_LEVEL_POSITION;
    }

    public double getLiftExtendPosition() {
        switch (hubLevel) {
            case THIRD:
                return LIFT_DUMP_POSITION;
            case SECOND:
                return LIFT_SECOND_LEVEL_POSITION;
            case FIRST:
                return LIFT_FIRST_LEVEL_POSITION;
            case SHARED:
                return LIFT_SHARED_HUB_POSITION;
        }
        return LIFT_DUMP_POSITION;
    }

    public void incrementArmOffset() {
        armOffset += 0.05;
    }

    public void decrementArmOffset() {
        armOffset -= 0.05;
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

    public boolean isLiftExtended() {
        return Math.abs(liftPidController.getLastError()) < 1;
    }

    public MotionProfile generateArmMotionProfile(double initialPosition, double endPosition) {
        return MotionProfileGenerator.generateSimpleMotionProfile(
                new MotionState(initialPosition, 0, 0, 0),
                new MotionState(endPosition, 0, 0, 0),
                ARM_MAX_VEL,
                ARM_MAX_ACCEL
        );
    }

    public void resetLift() {
        liftMode = LiftMode.RESET;
    }

    public OuttakeState getOuttakeState() {
        return outtakeState;
    }

    public void autoState() {
        outtakeState = OuttakeState.AUTO;
    }

    @Override
    public void update(TelemetryPacket packet) {
        switch (outtakeState) {
            case AUTO:
                lockerPosition = LOCKER_LOCK_POSITION;
                dumpPosition = DUMPER_AUTO_POSITION;
                armPosition = ARM_RETRACT_POSITION;
                break;
            case AUTO_EXTEND:
                lockerPosition = LOCKER_LOCK_POSITION;
                dumpPosition = DUMPER_AUTO_POSITION;
                if (clock.seconds() - initialTimestamp >= LIFT_EXTEND_TIME && liftPidController.getTargetPosition() != getLiftExtendPosition()) {
                    liftPidController.setTargetPosition(getLiftExtendPosition());
                }
                if (!armMoving) {
                    armMoving = true;
                    initialTimestamp = clock.seconds();
                    armMotionProfile = generateArmMotionProfile(ARM_RETRACT_POSITION, getArmExtendPosition());
                }
                if (armMoving) {
                    if (clock.seconds() - initialTimestamp >= armMotionProfile.duration()) {
                        armMoving = false;
                        outtakeState = OuttakeState.EXTEND;
                        liftPidController.setTargetPosition(getLiftExtendPosition());
                    } else {
                        armPosition = armMotionProfile.get(clock.seconds() - initialTimestamp).getX();
                        if (armPosition > ARM_LIFT_POSITION) {
                            dumpPosition = DUMPER_DUMP_POSITION;
                        } else {
                            dumpPosition = DUMPER_AUTO_POSITION;
                        }
                    }
                }
                break;
            case RETRACT:
                lockerPosition = LOCKER_HOLDING_POSITION;
                dumpPosition = DUMPER_RETRACT_POSITION;
                armPosition = ARM_PICK_FREIGHT_POSITION;
                break;
            case PICK_FREIGHT:
                armPosition = ARM_PICK_FREIGHT_POSITION;
                lockerPosition = LOCKER_UNLOCK_POSITION;
                if (clock.seconds() - initialTimestamp >= ARM_EXTEND_TIME * 0.5)
                    lockerPosition = LOCKER_LOCK_POSITION;
                if (clock.seconds() - initialTimestamp >= ARM_EXTEND_TIME) {
                    outtakeState = OuttakeState.EXTEND_ARM;
                    initialTimestamp = clock.seconds();
                }
                break;
            case EXTEND_ARM:
                lockerPosition = LOCKER_LOCK_POSITION;
                dumpPosition = DUMPER_LIFTING_POSITION;
                if (clock.seconds() - initialTimestamp >= LIFT_EXTEND_TIME && liftPidController.getTargetPosition() != LIFT_DUMP_POSITION) {
                    liftPidController.setTargetPosition(getLiftExtendPosition());
                }
                if (!armMoving) {
                    armMoving = true;
                    initialTimestamp = clock.seconds();
                    armMotionProfile = generateArmMotionProfile(ARM_PICK_FREIGHT_POSITION, getArmExtendPosition());
                }
                if (armMoving) {
                    if (clock.seconds() - initialTimestamp >= armMotionProfile.duration()) {
                        armMoving = false;
                        outtakeState = OuttakeState.EXTEND;
                        liftPidController.setTargetPosition(getLiftExtendPosition());
                    } else {
                        armPosition = armMotionProfile.get(clock.seconds() - initialTimestamp).getX();
                        if (armPosition > ARM_LIFT_POSITION) {
                            dumpPosition = DUMPER_DUMP_POSITION;
                        } else {
                            dumpPosition = DUMPER_LIFTING_POSITION;
                        }
                    }
                }
                break;
            case EXTEND:
                dumpPosition = DUMPER_DUMP_POSITION;
                /*
                if (hubLevel == HubLevel.FIRST)
                    dumpPosition = DUMPER_FIRST_LEVEL_POSITION;
                 */
                if (hubLevel == HubLevel.SHARED)
                    dumpPosition = DUMPER_SHARED_HUB_POSITION;
                lockerPosition = LOCKER_LOCK_POSITION;
                armPosition = getArmExtendPosition();
                armPosition += armOffset;
                break;
            case DUMP:
                dumpPosition = DUMPER_DUMP_POSITION;
                /*
                if (hubLevel == HubLevel.FIRST)
                    dumpPosition = DUMPER_FIRST_LEVEL_POSITION;
                 */
                if (hubLevel == HubLevel.SHARED)
                    dumpPosition = DUMPER_SHARED_HUB_POSITION;
                lockerPosition = LOCKER_UNLOCK_POSITION;
                armPosition = getArmExtendPosition();
                armPosition += armOffset;
                break;
            case RETRACT_ARM:
                dumpPosition = DUMPER_LIFTING_POSITION;
                lockerPosition = LOCKER_LOCK_POSITION;
                armPosition = ARM_LIFT_POSITION;
                if (!armMoving) {
                    armMoving = true;
                    initialTimestamp = clock.seconds();
                }
                if (armMoving && clock.seconds() - initialTimestamp >= ARM_EXTEND_TIME) {
                    armPosition = ARM_PICK_FREIGHT_POSITION;
                }
                if (armMoving && clock.seconds() - initialTimestamp >= 2*ARM_EXTEND_TIME) {
                    armMoving = false;
                    outtakeState = OuttakeState.RETRACT;
                    liftPidController.setTargetPosition(LIFT_RETRACT_POSITION);
                }
                break;
        }
        packet.put("Outtake State", outtakeState);
        switch (liftMode) {
            case PID:
                lift.setPower(liftPidController.update(getLiftPosition()));
                break;
            case MANUAL:
                lift.setPower(liftPower);
                break;
            case RESET:
                lift.setPower(-0.7);
                if (limitSwitch.isPressed()) {
                    lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    liftMode = LiftMode.PID;
                }
                break;
        }
        packet.put("Limit Switch Pressed", limitSwitch.isPressed());
        arm1.setPosition(armPosition + ARM_OFFSET);
        arm2.setPosition(armPosition);
        dump.setPosition(dumpPosition);
        locker.setPosition(lockerPosition);
    }
}
