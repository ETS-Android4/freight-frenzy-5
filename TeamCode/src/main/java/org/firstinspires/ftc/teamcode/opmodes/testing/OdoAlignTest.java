package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.framework.Robot;
import org.firstinspires.ftc.teamcode.subsystems.OdometryWheels;
import org.firstinspires.ftc.teamcode.subsystems.SimpleMecanumDrive;

@Disabled
@TeleOp
public class OdoAlignTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        Robot robot = new Robot(this);
        SimpleMecanumDrive drive = new SimpleMecanumDrive(robot);
        robot.registerSubsystem(drive);
        OdometryWheels odometryWheels = new OdometryWheels(robot);
        odometryWheels.setPoseEstimate(new Pose2d(9,64,0));
        PIDFController headingPID = new PIDFController(new PIDCoefficients(1.5, 0, 0));
        PIDFController distancePID = new PIDFController(new PIDCoefficients(0.3, 0, 0));
        headingPID.setTargetPosition(0);
        distancePID.setTargetPosition(63.5);
        waitForStart();
        while (!isStopRequested()) {
            odometryWheels.update();
            robot.update();
            Pose2d poseEstimate = odometryWheels.getPoseEstimate();
            drive.setDrivePower(new Pose2d(0, distancePID.update(poseEstimate.getY()), headingPID.update(imu.getAngularOrientation().firstAngle)));
            telemetry.addData("Estimated Pose", poseEstimate);
            telemetry.addData("Distance PID", distancePID.update(poseEstimate.getY()));
            telemetry.addData("IMU Info", imu.getAngularOrientation());
            telemetry.addData("Heading PID", headingPID.update(imu.getAngularOrientation().firstAngle));
            telemetry.update();
        }
    }
}
