package org.firstinspires.ftc.teamcode.vision;

import android.util.Pair;

import androidx.annotation.NonNull;

import org.jetbrains.annotations.Contract;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.concurrent.ArrayBlockingQueue;

public class PropPipeline extends OpenCvPipeline {
    private static float CAMERA_WIDTH;
    private static float CAMERA_HEIGHT;

    public static final Scalar red = new Scalar(255,0,0);

    //These are red
    public static double topRectWidthPercentage = 0.4;
    public static double topRectHeightPercentage = 0.54;
    public static double redWidthPercent = topRectWidthPercentage;
    public static double redHeightPercent = topRectHeightPercentage;
    public static double blueness = 2.75;
    //The points needed for the rectangles are calculated here
    public static int rectangleHeight = 10;
    //The width and height of the rectangles in terms of pixels
    public static int rectangleWidth = 10;
    private boolean running = false;
    private Pair<Integer, Integer> curRun = new Pair<>(-1, 0), greatestConfidence = new Pair<>(-1, 0);
    private final ArrayBlockingQueue<Integer> queue;
    private int totalTimesRan = 0;
    private final boolean debug;

    public PropPipeline(boolean isRed, boolean debug, ArrayBlockingQueue<Integer> queue) {
        if (!debug && isRed) {
            topRectWidthPercentage = redWidthPercent;
            topRectHeightPercentage = redHeightPercent;
        }
        this.debug = debug;
        this.queue = queue;
    }

    public void reset() {
        running = true;
        totalTimesRan = 0;
        curRun = new Pair<>(-1, 0);
        greatestConfidence = new Pair<>(-1, 0);
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

    @NonNull
    @Contract(value = " -> new", pure = true)
    public static Rect getRect() {
        return new Rect(
                (int) (CAMERA_WIDTH * topRectWidthPercentage),
                (int) (CAMERA_HEIGHT * topRectHeightPercentage),
                rectangleWidth,
                rectangleHeight
        );
    }

    /**
     * @param input input frame matrix
     */
    @Override
    public synchronized Mat processFrame(Mat input) {
        if (running) {
            final Rect rect = getRect();
            Mat rgbMat = input.submat(rect);
            Mat redMat = new Mat(), greenMat = new Mat(), blueMat = new Mat();
            Core.extractChannel(rgbMat, redMat, 0);
            Core.extractChannel(rgbMat, greenMat, 1);
            Core.extractChannel(rgbMat, blueMat, 2);

            final double red = Core.mean(redMat).val[0];
            final double blue = Core.mean(blueMat).val[0] * blueness;

//            final int pos = getIndexOfMaxOf3Params(red, green, blue);
//            curRun = new Pair<>(pos, pos == curRun.first ? curRun.second + 1 : 0);
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
