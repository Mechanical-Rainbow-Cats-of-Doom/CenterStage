package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class HighlightSelectionZonePipeline extends OpenCvPipeline {
    private static final Scalar color = new Scalar(255,0,0);
    private static final Scalar detectedColor = new Scalar(0,255,0);
    private final boolean isRed;
    private final PropPipeline.PropPipelineRectsProvider rects;

    public HighlightSelectionZonePipeline(boolean isRed, PropPipeline.PropPipelineRectsProvider rects) {
        this.isRed = isRed;
        this.rects = rects;
    }

    @Override
    public Mat processFrame(Mat input) {
        Rect[] rects = this.rects.rects();
        int pos = PropPipeline.findPositionWithConfidence(input, rects, isRed).first;
        for (int i = 0; i < rects.length; i++) {
            Scalar color = HighlightSelectionZonePipeline.color;
            if(i == pos) {
                color = HighlightSelectionZonePipeline.detectedColor;
            }
            Imgproc.rectangle(input, rects[i], color, 1);
        }

        return input;
    }
}
