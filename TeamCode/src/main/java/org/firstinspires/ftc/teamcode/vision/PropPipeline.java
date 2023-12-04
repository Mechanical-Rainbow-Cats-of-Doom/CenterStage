package org.firstinspires.ftc.teamcode.vision;

import android.util.Pair;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Pose2d;

import org.jetbrains.annotations.Contract;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.concurrent.ArrayBlockingQueue;

public class PropPipeline extends OpenCvPipeline {
    public interface PropPipelineRectsProvider {
        Rect[] rects();
    }

    @Config
    public static class PropPipelineConfig implements PropPipelineRectsProvider {
        public static double x1, y1, width1, height1;
        public static double x2, y2, width2, height2;

        public static double x3, y3, width3, height3;
        public Rect[] rects() {
            return new Rect[] {
                    createRectDecimal(x1, y1, width1, height1),
                    createRectDecimal(x2, y2, width2, height2),
                    createRectDecimal(x3, y3, width3, height3),
            };
        }

    }

    public static class BasicPropPipelineRects implements PropPipelineRectsProvider {
        private final Rect[] rects;

        public BasicPropPipelineRects(Rect... rects) {
            this.rects = rects;
        }

        @Override
        public Rect[] rects() {
            return rects;
        }
    }

    private boolean running = false;
    private Pair<Integer, Double> curRun = new Pair<>(-1, Double.NEGATIVE_INFINITY),
            greatestConfidence = new Pair<>(-1, Double.NEGATIVE_INFINITY);
    private final ArrayBlockingQueue<Integer> queue;
    private int totalTimesRan = 0;
    private final boolean debug;
    private final boolean isRed;
    private final PropPipelineRectsProvider propPipelineRectsProvider;

    public PropPipeline(boolean isRed, boolean debug, ArrayBlockingQueue<Integer> queue,
                        PropPipelineRectsProvider propPipelineRectsProvider) {
        this.isRed = isRed;
        this.debug = debug;
        this.queue = queue;
        this.propPipelineRectsProvider = propPipelineRectsProvider;
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
            curRun = findPositionWithConfidence(input, propPipelineRectsProvider.rects(), isRed);

            if (curRun.second > greatestConfidence.second) {
                greatestConfidence = curRun;
            }
            totalTimesRan++;
            if (totalTimesRan >= 6) {
                queue.offer(greatestConfidence.first);
                running = false;
            }
        }
        return input;
    }

    @NonNull
    @Contract("_, _, _ -> new")
    public static Pair<Integer, Double> findPositionWithConfidence(Mat input, @NonNull Rect[] rects, boolean isRed) {
        int pos = 0;
        double confidence = 0;
        Mat componentMat = new Mat(), nonComponentMat = new Mat(), greenMat = new Mat();
        Core.extractChannel(input, nonComponentMat, isRed ? 2 : 0);
        Core.extractChannel(input, componentMat, isRed ? 0 : 2);
        Core.extractChannel(input, greenMat, 1);
        for (int i = 0; i < rects.length; i++) {
            final double component = Core.mean(componentMat.submat(rects[i])).val[0];
            final double noncomponent = Core.mean(nonComponentMat.submat(rects[i])).val[0];
            final double green = Core.mean(greenMat.submat(rects[i])).val[0];
            final double totalConfidence = component / (component + noncomponent + green);

            if(totalConfidence > confidence) {
                pos = i;
                confidence = totalConfidence;
            }
        }

        return new Pair<>(pos, confidence);
    }

    @NonNull
    public static Rect createRectDecimal(double x, double y, double width, double height) {
        int CAMERA_WIDTH = PropDetector.CAMERA_WIDTH;
        int CAMERA_HEIGHT = PropDetector.CAMERA_HEIGHT;

        int newX = (int) (Math.max(Math.min(x, 1),0)*CAMERA_WIDTH);
        int newY = (int) (Math.max(Math.min(y, 1),0)*CAMERA_HEIGHT);

        int newWidth = Math.min(CAMERA_WIDTH-newX, (int)(Math.max(0,width)*CAMERA_WIDTH));
        int newHeight = Math.min(CAMERA_HEIGHT-newY, (int)(Math.max(0,height)*CAMERA_HEIGHT));

        return new Rect(newX, newY, newWidth, newHeight);
    }
}
