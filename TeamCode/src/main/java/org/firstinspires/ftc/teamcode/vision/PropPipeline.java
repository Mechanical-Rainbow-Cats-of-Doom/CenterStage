package org.firstinspires.ftc.teamcode.vision;

import android.util.Pair;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;

import org.jetbrains.annotations.Contract;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.concurrent.ArrayBlockingQueue;

@Config
public class PropPipeline extends OpenCvPipeline {
    public static double TIMES_TO_RUN_WITHOUT_VALID = 5;
    public static double TIMES_TO_RUN_WITH_VALID = 3;
    public static double RED_CONFIDENCE_THRESHOLD = 0.4;
    public static double BLUE_CONFIDENCE_THRESHOLD = 0.4;

    public interface PropPipelineRectsProvider {
        Rect[] rects();

        enum Default implements PropPipelineRectsProvider {
            RED_BOARD_SIDE(createRectDecimal(0.33, 0.84, 0.15, 0.1),
                    createRectDecimal(0.84, 0.45, 0.15, 0.1)),
            RED_AUDIENCE_SIDE(createRectDecimal(0, 0.18, 0.12, 0.2),
                    createRectDecimal(0.47, 0.14, 0.15, 0.1)),
            BLUE_BOARD_SIDE(createRectDecimal(0, 0.15, 0.08, 0.14),
                    createRectDecimal(0.45, 0.15, 0.15, 0.1)),
            BLUE_AUDIENCE_SIDE(createRectDecimal(0.36, 0.14, 0.15, 0.1),
                    createRectDecimal(0.9, 0.3, 0.1, 0.1));

            private final Rect[] rects;

            Default(Rect... rects) {
                this.rects = rects;
            }

            @Override
            public Rect[] rects() {
                return rects;
            }
        }

        @Config
        class PropPipelineDashboardConfig implements PropPipelineRectsProvider {
            public static double x1, y1, width1 = .2, height1 = .2;
            public static double x2 = .2, y2, width2 = .2, height2 = .2;
            public boolean hasThirdRectangle = false;
            public static double x3 = .4, y3, width3 = .2, height3 = .2;
            public static String defaultName = "";
            public Rect[] rects() {
                try {
                    Default defaultObject = Default.valueOf(defaultName);
                    return defaultObject.rects();
                } catch (IllegalArgumentException ignored) {}
                return (hasThirdRectangle) ? new Rect[] {
                        createRectDecimal(x1, y1, width1, height1),
                        createRectDecimal(x2, y2, width2, height2),
                        createRectDecimal(x3, y3, width3, height3),
                } : new Rect[] {
                        createRectDecimal(x1, y1, width1, height1),
                        createRectDecimal(x2, y2, width2, height2),
                };
            }

        }

        class BasicPropPipelineRects implements PropPipelineRectsProvider {
            private final Rect[] rects;

            public BasicPropPipelineRects(Rect... rects) {
                this.rects = rects;
            }

            @Override
            public Rect[] rects() {
                return rects;
            }
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

            if (++totalTimesRan >= (greatestConfidence.first == -1 ? TIMES_TO_RUN_WITHOUT_VALID : TIMES_TO_RUN_WITH_VALID)) {
                queue.offer(greatestConfidence.first);
                running = false;
            }
        }
        return input;
    }

    @NonNull
    @Contract("_, _, _ -> new")
    public static Pair<Integer, Double> findPositionWithConfidence(@NonNull Mat input, @NonNull Rect[] rects, boolean isRed) {
        int pos = -1;
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

            if (totalConfidence >= (isRed ? RED_CONFIDENCE_THRESHOLD : BLUE_CONFIDENCE_THRESHOLD) && totalConfidence > confidence) {
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
