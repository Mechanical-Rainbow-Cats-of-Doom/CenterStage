package org.firstinspires.ftc.teamcode.vision;

import android.util.Pair;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.concurrent.ArrayBlockingQueue;

public class PropPipeline extends OpenCvPipeline {

    public static final Scalar red = new Scalar(255,0,0);

    float x1, y1, width1, height1;
    float x2, y2, width2, height2;
    float x3, y3, width3, height3;

    public Rect[] rects() {
        // TODO set these better
        float CAMERA_WIDTH = 320;
        float CAMERA_HEIGHT = 240;
        return new Rect[] {
            new Rect((int) (x1*CAMERA_WIDTH), (int) (y1*CAMERA_HEIGHT), (int) (width1*CAMERA_WIDTH),
                    (int) (height1*CAMERA_HEIGHT)),
            new Rect((int) (x2*CAMERA_WIDTH), (int) (y2*CAMERA_HEIGHT), (int) (width2*CAMERA_WIDTH),
                    (int) (height2*CAMERA_HEIGHT)),
            new Rect((int) (x3*CAMERA_WIDTH), (int) (y3*CAMERA_HEIGHT), (int) (width3*CAMERA_WIDTH),
                    (int) (height3*CAMERA_HEIGHT)),
        };
    }

    // use in case of low blue levels
    public static double blueness = 1;
    private boolean running = false;
    private Pair<Integer, Double> curRun = new Pair<>(-1, 0d),
            greatestConfidence = new Pair<>(-1, 0d);
    private final ArrayBlockingQueue<Integer> queue;
    private int totalTimesRan = 0;
    private final boolean debug;
    private final boolean isRed;

    public PropPipeline(boolean isRed, boolean debug, ArrayBlockingQueue<Integer> queue) {
        this.isRed = isRed;
        this.debug = debug;
        this.queue = queue;
    }

    public void reset() {
        running = true;
        totalTimesRan = 0;
        curRun = new Pair<>(-1, 0d);
        greatestConfidence = new Pair<>(-1, 0d);
    }

    public void startPipeline() {
        running = true;
    }

    public void interruptPipeline() {
        running = false;
    }

    public boolean isRunning() {
        return running;
    }

    /**
     * @param input input frame matrix
     */
    @Override
    public synchronized Mat processFrame(Mat input) {
        if (running) {
            int pos = 0;
            double confidence = 0;
            Rect[] rects = rects();
            for (int i = 0; i < rects.length; i++) {
                Mat rgbMat = input.submat(rects[i]);
                Mat redMat = new Mat(), blueMat = new Mat();
                Core.extractChannel(rgbMat, redMat, 0);
                Core.extractChannel(rgbMat, blueMat, 2);

                final double red = Core.mean(redMat).val[0];
                final double blue = Core.mean(blueMat).val[0] * blueness;

                final boolean value = (red < blue) ^ isRed;
                final double singleConfidence = Math.abs(red - blue);

                if(value && singleConfidence > confidence) {
                    pos = i;
                    confidence = singleConfidence;
                }
            }

            curRun = new Pair<>(pos, confidence);
            if (curRun.second > greatestConfidence.second) {
                greatestConfidence = curRun;
            }
            totalTimesRan++;
            if (greatestConfidence.second >= 3 || totalTimesRan >= 6) {
                queue.offer(greatestConfidence.first);
                running = false;
            }
        }
        return input;
    }



    /**
     * Draw the rectangle onto the desired mat
     * @param mat   The mat that the rectangle should be drawn on
     * @param rect  The rectangle
     * @param color The color the rectangle will be
     */
    public static void drawRectOnToMat(Mat mat, Rect rect, Scalar color) {
        Imgproc.rectangle(mat, rect, color, 1);
    }
}
