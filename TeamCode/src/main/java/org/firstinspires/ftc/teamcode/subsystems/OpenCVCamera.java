package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.framework.Robot;
import org.firstinspires.ftc.teamcode.framework.Subsystem;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
public class OpenCVCamera implements Subsystem {
    private OpenCvWebcam webcam;

    public OpenCVCamera (Robot robot) {
        HardwareMap hardwareMap = robot.getHardwareMap();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

    }

    @Override
    public void update(TelemetryPacket packet) {

    }
}
