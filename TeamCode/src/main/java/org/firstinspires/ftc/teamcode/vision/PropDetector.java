package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.Random;
import java.util.concurrent.ArrayBlockingQueue;

import androidx.annotation.NonNull;

public class PropDetector {
    private final OpenCvCamera camera;
    private final boolean debug, isRed;
    public static int CAMERA_WIDTH = 320, CAMERA_HEIGHT = 240;
    public static OpenCvCameraRotation ORIENTATION = OpenCvCameraRotation.UPRIGHT;
    private final ArrayBlockingQueue<Integer> visionVals = new ArrayBlockingQueue<>(1);
    private final PropPipeline.PropPipelineRectsProvider rects;
    public PropDetector(@NonNull HardwareMap hMap, String webcamName, boolean debug, boolean isRed,
                        PropPipeline.PropPipelineRectsProvider rects) {
        this.isRed = isRed;
        this.debug = debug;
        this.rects = rects;

        OpenCvCameraFactory cameraFactory = OpenCvCameraFactory.getInstance();
        int cameraMonitorViewId = hMap
                .appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hMap.appContext.getPackageName());

        camera = cameraFactory.createWebcam(hMap.get(WebcamName.class, webcamName), cameraMonitorViewId); //for configurating remove isred from here
        camera.setPipeline(new HighlightSelectionZonePipeline(isRed, rects));
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, ORIENTATION);
                if (debug) {
                    FtcDashboard.getInstance().startCameraStream(camera, 10);
                }
            }

            @Override
            public void onError(int errorCode) {
                System.out.println("OpenCv Pipeline error with error code " + errorCode);
            }
        });
    }

    /**
     * Resets pipeline on call
     * Stalls code until pipeline is done with figuring out (max time of around 0.33 seconds)
     *
     * @param runnable program to run while waiting
     * @return integer 0 - 2, corresponds to left, center, and right respectively
     */
    public synchronized int run(Runnable runnable) throws InterruptedException {
        final PropPipeline pipeline = new PropPipeline(isRed, debug, visionVals, rects);
        camera.setPipeline(pipeline);
        pipeline.startPipeline();
        while(visionVals.peek() == null) {
            runnable.run();
        }
        final int output = visionVals.take();
        if (output == -1) {
            System.out.println("something fucked up real bad, vision didn't return val 🐶🐱");
            return new Random().nextInt(3);
        } else return output;
    }

    /**
     * Resets pipeline on call
     * Stalls code until pipeline is done with figuring out (max time of around 0.33 seconds)
     *
     * @return integer 0 - 2, corresponds to left, center, and right respectively
     */
    public synchronized int run() throws InterruptedException {
        return run(() -> {});
    }

    public void reset() {
        camera.setPipeline(new HighlightSelectionZonePipeline(isRed, rects));
    }

}