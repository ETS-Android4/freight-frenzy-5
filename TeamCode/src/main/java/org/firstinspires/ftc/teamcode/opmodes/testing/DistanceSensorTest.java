package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.framework.Robot;
import org.firstinspires.ftc.teamcode.subsystems.SimpleMecanumDrive;

@TeleOp
public class DistanceSensorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RevColorSensorV3 sensor = hardwareMap.get(RevColorSensorV3.class, "distance");
        RevColorSensorV3 sensor2 = hardwareMap.get(RevColorSensorV3.class, "distance2");
        Robot robot = new Robot(this);
        SimpleMecanumDrive drive = new SimpleMecanumDrive(robot);
        robot.registerSubsystem(drive);
        PIDFController headingPID = new PIDFController(new PIDCoefficients(0.1, 0, 0));
        PIDFController distancePID = new PIDFController(new PIDCoefficients(-0.15, 0, 0));
        headingPID.setTargetPosition(0);
        distancePID.setTargetPosition(35);
        waitForStart();
        while (!isStopRequested()) {
            double distance1 = sensor.getDistance(DistanceUnit.MM);
            double distance2 = sensor2.getDistance(DistanceUnit.MM) + 2;
            double headingMeasurement = (distance1 - distance2) / 2;
            double lateralMeasurement = (distance1 + distance2) / 2;

            drive.setDrivePower(new Pose2d(0, distancePID.update(lateralMeasurement), headingPID.update(headingMeasurement)));

            telemetry.addData("Distance 1", distance1);
            telemetry.addData("Distance 2", distance2);
            telemetry.addData("Heading Measurement", headingMeasurement);
            telemetry.addData("Lateral Measurement", lateralMeasurement);
            telemetry.update();
            robot.update();
        }
    }
}
