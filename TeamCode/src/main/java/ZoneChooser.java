import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

public class ZoneChooser extends OpenCvPipeline {
    private final double THRESHOLD = 12;

    private Mat mat = new Mat();
    private Mat lowerMat;
    private Mat upperMat;
    private Rect upperROI = new Rect(new Point(240, 120), new Point(304, 145));
    private Rect lowerROI = new Rect(new Point(240, 145), new Point(304, 170));
    private Target target;
    private Telemetry telemetry;
    private OpenCvCamera cam;

    public ZoneChooser(HardwareMap hwMap, Telemetry t) {
        telemetry = t;
        int camMonViewId = hwMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId",
                "id",
                hwMap.appContext.getPackageName()
        );
        cam = OpenCvCameraFactory.getInstance().createWebcam(
                hwMap.get(WebcamName.class, "Webcam 1"),
                camMonViewId
        );
        cam.setPipeline(this);
        cam.openCameraDeviceAsync(
                () -> cam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT)
        );
    }

    @Override
    public Mat processFrame(Mat input) {
        // -- THRESHOLDING --
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGBA2RGB);
        Imgproc.cvtColor(mat, mat, Imgproc.COLOR_RGB2HSV);
        Scalar lowerBound = new Scalar(15.0 / 2, 100, 100);
        Scalar upperBound = new Scalar(45.0 / 2, 255, 255);
        Core.inRange(mat, lowerBound, upperBound, mat);
        // -- DIVIDE --
        upperMat = mat.submat(upperROI);
        lowerMat = mat.submat(lowerROI);
        // -- AVERAGE --
        double upperValue = Math.round(Core.mean(upperMat).val[2] / 255);
        double lowerValue = Math.round(Core.mean(lowerMat).val[2] / 255);
        upperMat.release();
        lowerMat.release();
        mat.release();
        // -- COMPARE --

        Scalar matchColor = new Scalar(0, 255, 0);
        Scalar mismatchColor = new Scalar(255, 0, 0);
        if (upperValue >= THRESHOLD) {
            target = Target.C;
            Imgproc.rectangle(input, upperROI, matchColor);
            Imgproc.rectangle(input, lowerROI, matchColor);
        }
        else if (lowerValue >= THRESHOLD) {
            target = Target.B;
            Imgproc.rectangle(input, upperROI, mismatchColor);
            Imgproc.rectangle(input, lowerROI, matchColor);
        }
        else {
            target = Target.A;
            Imgproc.rectangle(input, upperROI, mismatchColor);
            Imgproc.rectangle(input, lowerROI, mismatchColor);
        }

        telemetry.addData("Top", upperValue + "%");
        telemetry.addData("Top", lowerValue + "%");
        telemetry.addData("Threshold", THRESHOLD);
        telemetry.addData("Target Zone", target);

        return input;
    }

    public Target getTarget() {
        return target;
    }

    public void stop() {
        cam.closeCameraDeviceAsync(() -> {});
    }
}
