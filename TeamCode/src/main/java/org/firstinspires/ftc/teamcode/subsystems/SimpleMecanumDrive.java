package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.framework.Robot;
import org.firstinspires.ftc.teamcode.framework.Subsystem;
import org.firstinspires.ftc.teamcode.opmodes.MatchState;

@Config
public class SimpleMecanumDrive implements Subsystem {
    private DcMotorEx[] motors = new DcMotorEx[4];
    private DcMotorEx tank;
    private Servo lifterL, lifterR, lifterC;
    //private OdometryWheels odometryWheels;
    BNO055IMU imu;

    private Double[] powers = {0.0, 0.0, 0.0, 0.0};
    private double tankPower = 0.0;
    private double lifterPosition = lifterExtendPosition;
    private double centerLifterPosition = centerLifterExtendPosition;
    private PIDFController wallAlignPID;
    private PIDFController headingAlignPID;

    public static double lifterRetractPosition = 1;
    public static double lifterExtendPosition = 0.3;
    public static double centerLifterExtendPosition = 0.5;
    public static double centerLifterRetractPosition = 0.2;

    public boolean wallAlign = false;

    public SimpleMecanumDrive (Robot robot) {
        motors[0] = robot.getMotor("DriveLF");
        motors[1] = robot.getMotor("DriveLR");
        motors[2] = robot.getMotor("DriveRR");
        motors[3] = robot.getMotor("DriveRF");
        tank = robot.getMotor("frontEncoder");
        lifterL = robot.getServo("LifterL");
        lifterR = robot.getServo("LifterR");
        lifterC = robot.getServo("LifterC");
        /*
        odometryWheels = new OdometryWheels(robot);
        if (MatchState.CurrentAlliance == MatchState.Alliance.BLUE)
            odometryWheels.setPoseEstimate(new Pose2d(9,64,0));
        else
            odometryWheels.setPoseEstimate(new Pose2d(9,-64,0));

         */
        imu = robot.getIMU("imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;

        motors[0].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[1].setDirection(DcMotorSimple.Direction.REVERSE);
        //lifterR.setDirection(Servo.Direction.REVERSE);

        wallAlignPID = new PIDFController(new PIDCoefficients(0.3, 0, 0));
        if (MatchState.CurrentAlliance == MatchState.Alliance.BLUE)
            wallAlignPID.setTargetPosition(63.75);
        else
            wallAlignPID.setTargetPosition(-63.75);
        headingAlignPID = new PIDFController(new PIDCoefficients(1.5, 0, 0));
        headingAlignPID.setTargetPosition(0);

        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void setDrivePower(Pose2d drivePower) {
        /*
        if (wallAlign) {
            Pose2d poseEstimate = odometryWheels.getPoseEstimate();
            drivePower = new Pose2d(drivePower.getX(),
                    wallAlignPID.update(poseEstimate.getY()),
                    headingAlignPID.update(imu.getAngularOrientation().firstAngle)
            );
        }

         */

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
        centerLifterPosition = centerLifterExtendPosition;
    }

    public void retractOdometry() {
        lifterPosition = lifterRetractPosition;
        centerLifterPosition = centerLifterRetractPosition;
    }

    @Override
    public void update(TelemetryPacket packet) {
        //odometryWheels.update();
        for (int i = 0; i < 4; i++){
            motors[i].setPower(powers[i]);
        }
        tank.setPower(tankPower);
        lifterL.setPosition(lifterPosition);
        lifterR.setPosition(lifterPosition);
        lifterC.setPosition(centerLifterPosition);
    }
}