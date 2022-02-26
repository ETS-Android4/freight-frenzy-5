package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.framework.Command;
import org.firstinspires.ftc.teamcode.framework.Robot;
import org.firstinspires.ftc.teamcode.framework.Subsystem;

public class DuckSpinner implements Subsystem {
    private Servo spinner;

    private double spinnerPower = 0;
    private double servoPower = 0;
    private double prevTime = 0;
    private double prevPower = 0.55;

    private final double POWER_INCREASE_FACTOR = 0.0000000015;

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
            setSpinnerPower(0.5);
        }

        @Override
        public boolean isCompleted() {
            return clock.seconds() - initialTimestamp >= seconds;
        }
    }

    public DuckSpinner(Robot robot) {
        spinner = robot.getServo("duckSpinner");
    }

    public SpinDuckCommand spinDuck(double power, double seconds) {
        return new SpinDuckCommand(power, seconds);
    }

    public void setSpinnerPower(double power) { //input: 0 to 1
        double currentTime = System.nanoTime();
        double timeDiff = currentTime - prevTime;
        double powerIncrease = timeDiff * POWER_INCREASE_FACTOR;
        double maxPower = powerIncrease + Math.abs(prevPower);
        double sign;
        if (power < 0){
            sign = -1;
        }
        else{
            sign = 1;
        }
        double newPower = Math.min(maxPower, Math.abs(power));
        //update values
        prevTime = currentTime;
        prevPower = newPower; //newPower
        //set power
        spinnerPower = ((newPower * sign) + 1.0 ) / 2.0; //convert to 0/0.5/1 range
        Log.i("duckSpinner", "power:"+ spinnerPower + " previous: "+ prevPower +
                " increased: "+ powerIncrease + "sign: " + sign);
    }

    @Override
    public void update(TelemetryPacket packet) {
        spinner.setPosition(spinnerPower);
    }
}
