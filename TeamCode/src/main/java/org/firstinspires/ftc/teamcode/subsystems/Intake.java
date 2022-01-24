package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.framework.Robot;
import org.firstinspires.ftc.teamcode.framework.Subsystem;

public class Intake implements Subsystem {
    private DcMotorEx intake;
    private Rev2mDistanceSensor distanceSensor;

    private double intakePower = 0;

    public Intake(Robot robot) {
        intake = robot.getMotor("rightEncoder");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        distanceSensor = robot.getHardwareMap().get(Rev2mDistanceSensor.class, "tof");
    }

    public void setIntakePower(double power) {
        intakePower = power;
    }

    public boolean hasFreight() {
        return distanceSensor.getDistance(DistanceUnit.MM) < 100;
    }

    public double getIntakeCurrent() {
        return intake.getCurrent(CurrentUnit.AMPS);
    }

    @Override
    public void update(TelemetryPacket packet) {
        if (intakePower > 0 && hasFreight())
            intake.setPower(0);
        else
            intake.setPower(intakePower);
        packet.put("Intake Current", intake.getCurrent(CurrentUnit.AMPS));
    }
}
