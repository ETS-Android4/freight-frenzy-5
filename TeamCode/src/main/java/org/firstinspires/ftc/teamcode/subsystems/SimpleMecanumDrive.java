package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.framework.Robot;
import org.firstinspires.ftc.teamcode.framework.Subsystem;

public class SimpleMecanumDrive implements Subsystem {
    private DcMotorEx[] motors = new DcMotorEx[4];
    private DcMotorEx tank;
    private Servo lifterL, lifterR;

    private Double[] powers = {0.0, 0.0, 0.0, 0.0};
    private double tankPower = 0.0;
    private double lifterLPosition = 1, lifterRPosition = 0.01;

    public SimpleMecanumDrive (Robot robot) {
        motors[0] = robot.getMotor("DriveLF");
        motors[1] = robot.getMotor("DriveLR");
        motors[2] = robot.getMotor("DriveRR");
        motors[3] = robot.getMotor("DriveRF");
        tank = robot.getMotor("DriveTank");
        lifterL = robot.getServo("LifterL");
        lifterR = robot.getServo("LifterR");

        motors[0].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[1].setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setDrivePower(Pose2d drivePower) {
        powers[0] = drivePower.getX() - drivePower.getY() - drivePower.getHeading();
        powers[1] = drivePower.getX() + drivePower.getY() - drivePower.getHeading();
        powers[2] = drivePower.getX() - drivePower.getY() + drivePower.getHeading();
        powers[3] = drivePower.getX() + drivePower.getY() + drivePower.getHeading();
    }

    public void setTankPower(double tankPower) {
        this.tankPower = tankPower;
    }

    @Override
    public void update(TelemetryPacket packet) {
        for (int i = 0; i < 4; i++){
            motors[i].setPower(powers[i]);
        }
        tank.setPower(tankPower);
        lifterL.setPosition(lifterLPosition);
        lifterR.setPosition(lifterRPosition);
    }
}