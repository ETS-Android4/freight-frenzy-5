package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;

public class Pipeline extends OpenCvPipeline {
    public static int CV_THRESHOLD = 150;
    static final Scalar BLUE = new Scalar(0, 0, 255);
    static final int CONTOUR_LINE_THICKNESS = 2;
    public static double MIN_RECT_AREA = 0;
    static final Rect CROP_RECT = new Rect(0,200,200,150);
    static final Rect CROP_RECT_2 = new Rect(400,190,240,150);

    Mat croppedInput = new Mat();
    Mat YCrCb = new Mat();
    Mat Cb = new Mat();
    Mat thresholdMat = new Mat();
    Mat morphedThreshold = new Mat();
    Mat contoursOnPlainImageMat = new Mat();
    Mat rectsOnPlainImageMat = new Mat();

    Rect bestRect;


    Mat erodeElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(3, 3));
    Mat dilateElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(6, 6));

    void cropInput(Mat input)
    {
        /*
        switch (MatchState.CurrentAlliance) {
            case BLUE:
                croppedInput = new Mat(input, CROP_RECT);
                break;
            case RED:


         */
        croppedInput = new Mat(input, CROP_RECT);
    }


    void inputToCb(Mat input)
    {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb, Cb, 1);
    }

    void morphMask(Mat input, Mat output)
    {
        /*
         * Apply some erosion and dilation for noise reduction
         */

        Imgproc.erode(input, output, erodeElement);
        Imgproc.erode(output, output, erodeElement);

        Imgproc.dilate(output, output, dilateElement);
        Imgproc.dilate(output, output, dilateElement);
    }

    void cropInput(Mat input, Rect cropRect) {
        input = input.submat(cropRect);
    }

    @Override
    public void init(Mat firstFrame) {
        inputToCb(firstFrame);
    }

    @Override
    public Mat processFrame(Mat input) {
        cropInput(input);
        inputToCb(croppedInput);
        Imgproc.GaussianBlur(Cb, Cb, new Size(5,5),0);

        Imgproc.threshold(Cb, thresholdMat, CV_THRESHOLD, 255, Imgproc.THRESH_BINARY);
        morphMask(thresholdMat, morphedThreshold);

        ArrayList<MatOfPoint> contoursList = new ArrayList<>();
        Imgproc.findContours(morphedThreshold, contoursList, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_NONE);
        input.copyTo(contoursOnPlainImageMat);
        Imgproc.drawContours(contoursOnPlainImageMat, contoursList, -1, BLUE, CONTOUR_LINE_THICKNESS, 8);

        ArrayList<Rect> boundingBoxes = new ArrayList<>();
        for (MatOfPoint contour : contoursList) {
            boundingBoxes.add(Imgproc.boundingRect(contour));
        }
        Collections.sort(boundingBoxes, (rect, t1) -> (int) (t1.area() - rect.area()));
        croppedInput.copyTo(rectsOnPlainImageMat);
        if (boundingBoxes.size() > 0 && boundingBoxes.get(0).area() > MIN_RECT_AREA) {
            bestRect = boundingBoxes.get(0);
            Imgproc.rectangle(rectsOnPlainImageMat, bestRect.tl(), bestRect.br(), BLUE, CONTOUR_LINE_THICKNESS);
        } else {
            bestRect = null;
        }
        //Imgproc.rectangle(rectsOnPlainImageMat, CROP_RECT_2.tl(), CROP_RECT_2.br(), BLUE, CONTOUR_LINE_THICKNESS);
        /*
        for (Rect rect : boundingBoxes) {
            Imgproc.rectangle(rectsOnPlainImageMat, rect.tl(), rect.br(), BLUE, CONTOUR_LINE_THICKNESS);
        }
        */

        return rectsOnPlainImageMat;
    }

    public double getX(){
        if (bestRect != null) {
            return bestRect.x;
        }
        return 0;
    }

    public double getArea() {
        if (bestRect != null) {
            return bestRect.area();
        }
        return 0;
    }
}
