package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.teamcode.subsystems.OpenCVCamera;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class BlueMarkerPipeline extends OpenCvPipeline {
    private OpenCVCamera.DuckPosition position = OpenCVCamera.DuckPosition.NONE;

    static final Scalar BLUE = new Scalar(0, 0, 255);
    static final Scalar GREEN = new Scalar(0, 255, 0);

    static final Rect LEFT_RECT = new Rect(165,315,40,30);//new Rect(240,330,40,20);
    static final Rect MIDDLE_RECT = new Rect(285,325,40,30);//new Rect(350,330,40,20);
    static final Rect RIGHT_RECT = new Rect(395,325,40,30);//new Rect(450,325,40,20);

    Mat leftCb, middleCb, rightCb;
    Mat YCrCb = new Mat();
    Mat Cb = new Mat();
    int leftAvg, middleAvg, rightAvg;

    void inputToCb(Mat input)
    {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb, Cb, 2);
    }

    @Override
    public void init(Mat firstFrame) {
        inputToCb(firstFrame);
        leftCb = Cb.submat(LEFT_RECT);
        middleCb = Cb.submat(MIDDLE_RECT);
        rightCb = Cb.submat(RIGHT_RECT);
    }

    @Override
    public Mat processFrame(Mat input) {
        inputToCb(input);
        Imgproc.rectangle(input, LEFT_RECT, BLUE,2);
        Imgproc.rectangle(input, MIDDLE_RECT, BLUE,2);
        Imgproc.rectangle(input, RIGHT_RECT, BLUE,2);

        leftCb = Cb.submat(LEFT_RECT);
        middleCb = Cb.submat(MIDDLE_RECT);
        rightCb = Cb.submat(RIGHT_RECT);

        leftAvg = (int) Core.mean(leftCb).val[0];
        middleAvg = (int) Core.mean(middleCb).val[0];
        rightAvg = (int) Core.mean(rightCb).val[0];

        Imgproc.putText(input, Integer.toString(leftAvg), LEFT_RECT.tl(), 2, 1, BLUE);
        Imgproc.putText(input, Integer.toString(middleAvg), MIDDLE_RECT.tl(), 2, 1, BLUE);
        Imgproc.putText(input, Integer.toString(rightAvg), RIGHT_RECT.tl(), 2, 1, BLUE);

        if (leftAvg <= middleAvg && leftAvg <= rightAvg) {
            Imgproc.rectangle(input, LEFT_RECT, GREEN,2);
            position = OpenCVCamera.DuckPosition.LEFT;
        } else if (middleAvg <= leftAvg && middleAvg <= rightAvg) {
            Imgproc.rectangle(input, MIDDLE_RECT, GREEN,2);
            position = OpenCVCamera.DuckPosition.MIDDLE;
        } else {
            Imgproc.rectangle(input, RIGHT_RECT, GREEN,2);
            position = OpenCVCamera.DuckPosition.RIGHT;
        }
        return input;
    }

    public OpenCVCamera.DuckPosition getPosition() {
        return position;
    }
}
