package org.firstinspires.ftc.teamcode.framework;

public interface Command {
    void start();

    void update();

    void stop();

    boolean isCompleted();
}