package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.framework.Robot;
import org.firstinspires.ftc.teamcode.framework.Subsystem;

import java.util.List;

public class Camera implements Subsystem {
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private WebcamName webcamName;

    private Recognition duckRecognition;

    public static final String VUFORIA_LICENSE_KEY = "AWiNNND/////AAABmXs1dnUBs0nRthBOBC9Kyho3qpyfTGG2bd6vCAGPVkkqo1CjZrX4bGMzfqbqVBTTC0FbNe4v409zGxLeT35LOz17xAiE9za3L2h9QYfG/HXweWFwImnHP3nRd/4BoM5Sufel1Qj3l9nXNd05ddZrVY5lgyU04m6vsLjMoredoYrHBkdzr5RnN495Hjg+sdw0dM8/7Gnrd6nGDaXFtEHQS2LveGgpGzLDeXXP0bany+tGjV7BopDAMUSJxpp5IPgkigxQZiLYXuL736Xc/rXrbHyDQzDfVfYludH0XgCHoSNxNXg2qXXjrAUmhvApMPRceGTjMkRNyRiqoVA0EBFpvOkxBVFFAyFyXU1JjnNiFRcd";

    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_DM.tflite";
    private static final String[] LABELS = {
            "Duck",
            "Marker"
    };

    private static final double DUCK_MIDDLE_X = 130;
    private static final double DUCK_RIGHT_X = 430;
    private static final double DUCK_TOLERANCE = 100;

    public enum DuckPosition {
        LEFT,
        MIDDLE,
        RIGHT
    }

    /*
    middle x: 130
    middle y: 268
    right x: 430
    right y: 265
     */

    public Camera(Robot robot) {
        HardwareMap hardwareMap = robot.getHardwareMap();
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_LICENSE_KEY;
        parameters.cameraName = webcamName;
        parameters.useExtendedTracking = false;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.2f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);

        if (tfod != null) {
            tfod.activate();
        }

        FtcDashboard.getInstance().startCameraStream(tfod, 0);
    }

    public float getDuckX() {
        if (duckRecognition != null) {
            return duckRecognition.getLeft();
        }
        return 0;
    }

    public float getDuckY() {
        if (duckRecognition != null) {
            return duckRecognition.getBottom();
        }
        return 0;
    }

    public DuckPosition getDuckPosition() {
        double x = getDuckX();
        if (Math.abs(x - DUCK_MIDDLE_X) < DUCK_TOLERANCE)
            return DuckPosition.MIDDLE;
        else if (Math.abs(x - DUCK_RIGHT_X) < DUCK_TOLERANCE)
            return DuckPosition.RIGHT;
        else
            return DuckPosition.LEFT;
    }

    @Override
    public void update(TelemetryPacket packet) {
        if (tfod != null) {
            List<Recognition> recognitions = tfod.getUpdatedRecognitions();
            if (recognitions != null) {
                for (Recognition recognition : recognitions) {
                    if (recognition.getLabel().equals(LABELS[0])) {
                        duckRecognition = recognition;
                        break;
                    }
                }
            }
        }
    }

    public void shutdown() {
        if (tfod != null) tfod.shutdown();
    }
}
