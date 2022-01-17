package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.framework.Robot;

public class FreightFrenzyRobot extends Robot {
    public final Drivetrain drive;
    public final DuckSpinner duck;
    public final Intake intake;
    public final Lift lift;
    public FreightFrenzyRobot(LinearOpMode opMode) {
        super(opMode);
        drive = new Drivetrain(this);
        registerSubsystem(drive);
        duck = new DuckSpinner(this);
        registerSubsystem(duck);
        intake = new Intake(this);
        registerSubsystem(intake);
        lift = new Lift(this);
        registerSubsystem(lift);
    }
}
