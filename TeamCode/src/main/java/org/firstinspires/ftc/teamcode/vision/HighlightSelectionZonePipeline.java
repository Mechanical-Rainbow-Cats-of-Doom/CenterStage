package org.firstinspires.ftc.teamcode.vision;

import android.util.Pair;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import javax.annotation.Nullable;

public class HighlightSelectionZonePipeline extends OpenCvPipeline {
    private static final Scalar color = new Scalar(255,0,0);
    private static final Scalar detectedColor = new Scalar(0,255,0);
    private final boolean isRed;
    private final PropPipeline.PropPipelineRectsProvider rects;
    private int lastResult;
    private double lastConfidence;

    public HighlightSelectionZonePipeline(boolean isRed, PropPipeline.PropPipelineRectsProvider rects) {
        this.isRed = isRed;
        this.rects = rects;
    }

    @Override
    public Mat processFrame(Mat input) {
        Rect[] rects = this.rects.rects();
        Pair<Integer, Double> data = PropPipeline.findPositionWithConfidence(input, rects, isRed);
        int pos = data.first;
        for (int i = 0; i < rects.length; i++) {
            Scalar color = HighlightSelectionZonePipeline.color;
            if(i == pos) {
                color = HighlightSelectionZonePipeline.detectedColor;
            }
            Imgproc.rectangle(input, rects[i], color, 1);
        }
        lastResult = data.first;
        lastConfidence = data.second;

        return input;
    }

    public int getLastResult() {
        return lastResult;
    }

    public double getLastConfidence() {
        return lastConfidence;
    }
}
