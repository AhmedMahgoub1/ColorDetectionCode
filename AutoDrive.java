package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
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
import org.openftc.easyopencv.OpenCvWebcam;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Inside Blue Auto", group = "Camera Auto")
public class OptimizedAutoDrive extends LinearOpMode {

    OpenCvWebcam webcam1 = null;
    TeamPropDeterminationPipeline pipeline;
    TeamPropDeterminationPipeline.TeamPropPosition cur = TeamPropDeterminationPipeline.TeamPropPosition.LEFT;

    @Override
    public void runOpMode() {
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        pipeline = new TeamPropDeterminationPipeline();
        webcam1.setPipeline(pipeline);
        webcam1.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam1.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {
            }
        });
        double xMulti = 0.675;
        DcMotor FL = hardwareMap.dcMotor.get("FL");
        DcMotor BL = hardwareMap.dcMotor.get("BL");
        DcMotor FR = hardwareMap.dcMotor.get("FR");
        DcMotor BR = hardwareMap.dcMotor.get("BR");
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);
        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        int aUpPos = 2800;
        int aDownPos = 0;
        int aLevel1 = (int) (aUpPos * 0.55);
        DcMotor arm = hardwareMap.dcMotor.get("Arm");
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int eLevel0 = 0;
        int eLevel1 = 1100;
        int eLevel2 = 1150;
        int eLevel3 = 2100;
        DcMotor extend = hardwareMap.dcMotor.get("Extend");
        extend.setDirection(DcMotorSimple.Direction.REVERSE);
        extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extend.setTargetPosition(eLevel0);
        extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int stage = 0;
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Servo r = hardwareMap.servo.get("RGrab");
        Servo l = hardwareMap.servo.get("LGrab");
        l.setPosition(0);
        r.setPosition(.35);

        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Realtime analysis", pipeline.getAnalysis());
            telemetry.addData("left Value", pipeline.getLeft());
            telemetry.addData("Center Value", pipeline.getCenter());
            telemetry.addData("Right Value", pipeline.getRight());
            telemetry.update();
            sleep(50);
        }
        
        cur = pipeline.getAnalysis();
        telemetry.addData("Snapshot post-START analysis", cur);
        telemetry.update();

        while (isStarted() && opModeIsActive()) {
            switch (cur) {
                case LEFT: {
                    while (true) {
                        extend.setTargetPosition(eLevel2);
                        extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        extend.setPower(1);
                        telemetry.addData("Right Front Wheel", FR.getCurrentPosition());
                        telemetry.addData("Left Front Wheel", FL.getCurrentPosition());
                        telemetry.addData("Right back Wheel", BR.getCurrentPosition());
                        telemetry.addData("Left back Wheel", BL.getCurrentPosition());
                        telemetry.update();
                        if (BR.getCurrentPosition() < -6500 && FR.getCurrentPosition() < -6500) {
                            FL.setPower(0);
                            FR.setPower(0);
                            BR.setPower(0);
                            BL.setPower(0);
                            l.setPosition(0.37);
                            arm.setTargetPosition((int) (2530 * .55));
                            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            arm.setPower(1);
                            break;
                        } else {
                            FL.setPower(0.3);
                            FR.setPower(0.3);
                            BR.setPower(0.3);
                            BL.setPower(0.3);
                        }
                        sleep(2);
                    }
                    sleep(40000);
                    extend.setTargetPosition(0);
                    extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    extend.setPower(1);
                    sleep((int) (1000 * xMulti));
                    arm.setTargetPosition(0);
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    arm.setPower(1);
                    sleep((int) (1000 * xMulti));
                    sleep((int) (800 * xMulti));
                    break;
                }
                case RIGHT: {
                    FL.setPower(0.4);
                    FR.setPower(0.4);
                    BR.setPower(0.4);
                    BL.setPower(0.4);
                    sleep((int) (1500 * xMulti));
                    FL.setPower(.4);
                    FR.setPower(-0.1);
                    BR.setPower(-0.1);
                    BL.setPower(.4);
                    sleep((int) (2000 * xMulti));
                    FL.setPower(0);
                    FR.setPower(0);
                    BR.setPower(0);
                    BL.setPower(0);
                    sleep((int) (800 * xMulti));
                    FL.setPower(-.4);
                    FR.setPower(-.4);
                    BR.setPower(-.4);
                    BL.setPower(-.4);
                    sleep((int) (750 * xMulti));
                    FL.setPower(-1);
                    FR.setPower(1);
                    BR.setPower(1);
                    BL.setPower(-1);
                    sleep((int) (900 * (xMulti - 0.0075)));
                    FL.setPower(0.4);
                    FR.setPower(-.4);
                    BR.setPower(0.4);
                    BL.setPower(-0.4);
                    sleep((int) (1025 * xMulti));
                    FL.setPower(0.4);
                    FR.setPower(.4);
                    BR.setPower(0.4);
                    BL.setPower(0.4);
                    sleep((int) (1600 * xMulti));
                    arm.setTargetPosition(aLevel1);
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    arm.setPower(1);
                    sleep((int) (800 * xMulti));
                    FL.setPower(0);
                    FR.setPower(0);
                    BR.setPower(0);
                    BL.setPower(0);
                    extend.setTargetPosition(eLevel2);
                    extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    extend.setPower(1);
                    sleep((int) (4000 * xMulti));
                    extend.setTargetPosition(0);
                    extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    extend.setPower(1);
                    sleep((int) (1000 * xMulti));
                    arm.setTargetPosition(0);
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    arm.setPower(1);
                    sleep((int) (1000 * xMulti));
                    sleep((int) (800 * xMulti));
                }
                break;
                case CENTER: {
                    FL.setPower(.4);
                    FR.setPower(.4);
                    BR.setPower(.4);
                    BL.setPower(.4);
                    sleep((int) (2250 * xMulti));
                    FL.setPower(0);
                    FR.setPower(0);
                    BR.setPower(0);
                    BL.setPower(0);
                    sleep((int) (800 * xMulti));
                    FL.setPower(-.4);
                    FR.setPower(-.4);
                    BR.setPower(-.4);
                    BL.setPower(-.4);
                    sleep((int) (950 * xMulti));
                    FL.setPower(-1);
                    FR.setPower(1);
                    BR.setPower(1);
                    BL.setPower(-1);
                    sleep((int) (900 * (xMulti - 0.0075)));
                    FL.setPower(0.4);
                    FR.setPower(-.4);
                    BR.setPower(0.4);
                    BL.setPower(-0.4);
                    sleep((int) (1025 * xMulti));
                    FL.setPower(0.4);
                    FR.setPower(.4);
                    BR.setPower(0.4);
                    BL.setPower(0.4);
                    sleep((int) (1700 * xMulti));
                    arm.setTargetPosition(aLevel1);
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    arm.setPower(1);
                    sleep((int) (800 * xMulti));
                    FL.setPower(0);
                    FR.setPower(0);
                    BR.setPower(0);
                    BL.setPower(0);
                    extend.setTargetPosition(eLevel2);
                    extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    extend.setPower(1);
                    sleep((int) (4000 * xMulti));
                    extend.setTargetPosition(0);
                    extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    extend.setPower(1);
                    sleep((int) (1000 * xMulti));
                    arm.setTargetPosition(0);
                    arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    arm.setPower(1);
                    sleep((int) (1000 * xMulti));
                    sleep((int) (800 * xMulti));
                }
                break;
            }
            telemetry.addData("Snapshot post-START analysis", cur);
            telemetry.update();
        }
    }

    static class TeamPropDeterminationPipeline extends OpenCvPipeline {
        static final Scalar BLUE = new Scalar(255, 0, 0);
        static final Scalar RED = new Scalar(255, 0, 0);
        static final Scalar GREEN = new Scalar(0, 255, 0);
        static final int REGION_WIDTH = 100;
        static final int REGION_HEIGHT = 100;
        Point region1TopLeftAnchor = new Point(320 - REGION_WIDTH / 2, 240 - REGION_HEIGHT / 2);
        Point region1BottomRightAnchor = new Point(320 + REGION_WIDTH / 2, 240 + REGION_HEIGHT / 2);
        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int left = 0;
        int center = 0;
        int right = 0;
        int[] output = new int[3];
        int leftCount = 0;
        int centerCount = 0;
        int rightCount = 0;

        public enum TeamPropPosition {
            LEFT,
            CENTER,
            RIGHT
        }

        private TeamPropPosition position = TeamPropPosition.LEFT;

        @Override
        public Mat processFrame(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Cb = YCrCb.submat(new Rect(region1TopLeftAnchor, region1BottomRightAnchor));
            Core.extractChannel(Cb, region1_Cb, 0);
            left = Core.mean(region1_Cb).val[0];
            if (left < 90) {
                leftCount++;
            }
            region1_Cb.release();
            if (leftCount > 200) {
                position = TeamPropPosition.LEFT;
                leftCount = 0;
            }
            Imgproc.rectangle(input, region1TopLeftAnchor, region1BottomRightAnchor, BLUE, 2);
            return input;
        }

        public TeamPropPosition getAnalysis() {
            return position;
        }

        public int getLeft() {
            return left;
        }

        public int getCenter() {
            return center;
        }

        public int getRight() {
            return right;
        }
    }
}
