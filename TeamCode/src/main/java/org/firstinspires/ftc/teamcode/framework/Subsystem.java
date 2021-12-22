package org.firstinspires.ftc.teamcode.framework;


import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

public interface Subsystem {
    public void update(TelemetryPacket packet);
}