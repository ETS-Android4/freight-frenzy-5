package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.framework.Robot;
import org.firstinspires.ftc.teamcode.framework.Subsystem;

@Config
public class SimpleMecanumDrive implements Subsystem {
    private DcMotorEx[] motors = new DcMotorEx[4];
    private DcMotorEx tank;
    private Servo lifterL, lifterR;
    private DistanceSensor distance1, distance2;

    private Double[] powers = {0.0, 0.0, 0.0, 0.0};
    private double tankPower = 0.0;
    private double lifterPosition = lifterExtendPosition;
    private PIDFController wallAlignPID;
    private PIDFController headingAlignPID;

    public static double lifterRetractPosition = 1;
    public static double lifterExtendPosition = 0.9;

    public boolean wallAlign = false;

    public SimpleMecanumDrive (Robot robot) {
        motors[0] = robot.getMotor("DriveLF");
        motors[1] = robot.getMotor("DriveLR");
        motors[2] = robot.getMotor("DriveRR");
        motors[3] = robot.getMotor("DriveRF");
        tank = robot.getMotor("frontEncoder");
        lifterL = robot.getServo("LifterL");
        lifterR = robot.getServo("LifterR");
        distance1 = robot.getHardwareMap().get(DistanceSensor.class, "distance");
        distance2 = robot.getHardwareMap().get(DistanceSensor.class, "distance2");

        motors[0].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[1].setDirection(DcMotorSimple.Direction.REVERSE);
        lifterR.setDirection(Servo.Direction.REVERSE);

        wallAlignPID = new PIDFController(new PIDCoefficients(-0.15, 0, 0));
        wallAlignPID.setTargetPosition(35);
        headingAlignPID = new PIDFController(new PIDCoefficients(0.1, 0, 0));
        headingAlignPID.setTargetPosition(0);

        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void setDrivePower(Pose2d drivePower) {
        if (wallAlign) {
            drivePower = new Pose2d(drivePower.getX(),
                    wallAlignPID.update((distance1.getDistance(DistanceUnit.MM) + distance2.getDistance(DistanceUnit.MM)) / 2),
                    headingAlignPID.update((distance1.getDistance(DistanceUnit.MM) - distance2.getDistance(DistanceUnit.MM)) / 2));
        }

        powers[0] = drivePower.getX() - drivePower.getY() - drivePower.getHeading();
        powers[1] = drivePower.getX() + drivePower.getY() - drivePower.getHeading();
        powers[2] = drivePower.getX() - drivePower.getY() + drivePower.getHeading();
        powers[3] = drivePower.getX() + drivePower.getY() + drivePower.getHeading();
    }

    public void toggleWallAlign() {
        wallAlign = !wallAlign;
    }

    public void setTankPower(double tankPower) {
        this.tankPower = tankPower;
    }

    public void extendOdometry() {
        lifterPosition = lifterExtendPosition;
    }

    public void retractOdometry() {
        lifterPosition = lifterRetractPosition;
    }

    @Override
    public void update(TelemetryPacket packet) {
        for (int i = 0; i < 4; i++){
            motors[i].setPower(powers[i]);
        }
        tank.setPower(tankPower);
        lifterL.setPosition(lifterPosition);
        lifterR.setPosition(lifterPosition);
    }
}