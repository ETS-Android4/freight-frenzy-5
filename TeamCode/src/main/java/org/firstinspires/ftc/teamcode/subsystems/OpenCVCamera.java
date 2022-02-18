package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.framework.Robot;
import org.firstinspires.ftc.teamcode.framework.Subsystem;
import org.firstinspires.ftc.teamcode.vision.Pipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
public class OpenCVCamera implements Subsystem {
    private static final double DUCK_MIDDLE_X = 200;
    private static final double DUCK_RIGHT_X = 475;
    private static final double DUCK_TOLERANCE = 100;

    public enum DuckPosition {
        LEFT,
        MIDDLE,
        RIGHT
    }
    private OpenCvWebcam webcam;
    private Pipeline pipeline;

    public OpenCVCamera (Robot robot) {
        HardwareMap hardwareMap = robot.getHardwareMap();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new Pipeline();
        webcam.setPipeline(pipeline);
        webcam.setMillisecondsPermissionTimeout(2500);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                /*
                 * Tell the webcam to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
                 * supports streaming from the webcam in the uncompressed YUV image format. This means
                 * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
                 * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
                 *
                 * Also, we specify the rotation that the webcam is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
    }

    public DuckPosition getDuckPosition() {
        double x = pipeline.getX();
        if (Math.abs(x - DUCK_MIDDLE_X) < DUCK_TOLERANCE)
            return DuckPosition.MIDDLE;
        else if (Math.abs(x - DUCK_RIGHT_X) < DUCK_TOLERANCE)
            return DuckPosition.RIGHT;
        else
            return DuckPosition.LEFT;
    }

    public double getDuckX() {
        return pipeline.getX();
    }

    public double getRecognitionArea() {
        return pipeline.getArea();
    }

    public void shutdown() {
        webcam.stopStreaming();
    }

    @Override
    public void update(TelemetryPacket packet) {
        packet.put("Duck Position", getDuckPosition());
    }
}
