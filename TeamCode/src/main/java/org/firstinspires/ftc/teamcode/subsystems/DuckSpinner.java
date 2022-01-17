package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.framework.Command;
import org.firstinspires.ftc.teamcode.framework.Robot;
import org.firstinspires.ftc.teamcode.framework.Subsystem;

public class DuckSpinner implements Subsystem {
    private DcMotorEx spinner;

    private double spinnerPower = 0;

    public class SpinDuckCommand implements Command {
        NanoClock clock;
        double initialTimestamp;
        double seconds;
        double power;

        public SpinDuckCommand(double power, double seconds) {
            this.power = power;
            this.seconds = seconds;
            clock = NanoClock.system();
            initialTimestamp = clock.seconds();
        }

        @Override
        public void start() {
            setSpinnerPower(power);
        }

        @Override
        public void update() {

        }

        @Override
        public void stop() {
            setSpinnerPower(0);
        }

        @Override
        public boolean isCompleted() {
            return clock.seconds() - initialTimestamp >= seconds;
        }
    }

    public DuckSpinner(Robot robot) {
        spinner = robot.getMotor("leftEncoder");
        spinner.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public SpinDuckCommand spinDuck(double power, double seconds) {
        return new SpinDuckCommand(power, seconds);
    }

    public void setSpinnerPower(double power) {
        spinnerPower = power;
    }

    @Override
    public void update(TelemetryPacket packet) {
        spinner.setPower(spinnerPower);
    }
}
