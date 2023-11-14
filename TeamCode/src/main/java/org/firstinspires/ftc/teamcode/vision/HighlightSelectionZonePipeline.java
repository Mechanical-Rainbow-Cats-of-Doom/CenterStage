package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class HighlightSelectionZonePipeline extends OpenCvPipeline {
    private static final Scalar color = new Scalar(255,0,0);

    @Override
    public Mat processFrame(Mat input) {
        Rect[] rects = PropPipeline.PropPipelineConfig.rects();
        for (Rect rect : rects) {
            Imgproc.rectangle(input, rect, color, 1);
        }

        return input;
    }
}
