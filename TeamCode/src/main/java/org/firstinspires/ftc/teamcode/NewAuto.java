package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="NewAuto", group="test")
public class NewAuto extends LinearOpMode {

    DcMotor frontLeft;
    DcMotor rearLeft;
    DcMotor frontRight;
    DcMotor rearRight;
    DcMotor launcherLeft;
    DcMotor launcherRight;
    CRServo launcherArmL;
    CRServo launcherArmR;
    Servo pusherServo;
    Servo wobbleArm;
    Servo wobbleGrip;
    OpenCvCamera webcam;
    EasyOpenCVExample.SkystoneDeterminationPipeline pipeline;
    DigitalChannel homeSwitch;
    static int fourRingThreshold = 156;
    static int oneRingThreshold = 146;

    @Override
    public void runOpMode() {

        //Motors
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        rearLeft = hardwareMap.dcMotor.get("rearLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        rearRight = hardwareMap.dcMotor.get("rearRight");
        launcherLeft = hardwareMap.dcMotor.get("launcherLeft");
        launcherRight = hardwareMap.dcMotor.get("launcherRight");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        launcherRight .setDirection(DcMotorSimple.Direction.REVERSE);
        launcherLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcherRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Servos
        pusherServo = hardwareMap.servo.get("pusherServo");
        launcherArmL = hardwareMap.crservo.get("launcherArmL");
        launcherArmR = hardwareMap.crservo.get("launcherArmR");
        wobbleArm = hardwareMap.servo.get("wobbleArm");
        wobbleGrip = hardwareMap.servo.get("wobbleGrip");

        launcherArmL.setDirection(DcMotorSimple.Direction.REVERSE);

        //Sensors
        homeSwitch = hardwareMap.get(DigitalChannel.class, "homeSwitch");

        //Important Variables
        double launcherArmPosition;

        while (homeSwitch.getState() == true){
            launcherArmL.setPower(-0.25);
            launcherArmR.setPower(-0.25);
        }

        launcherArmL.setPower(0);
        launcherArmR.setPower(0);

        launcherArmPosition = 0.0;

        wobbleArm.setPosition(0.4);
        wobbleGrip.setPosition(0);
        pusherServo.setPosition(-1);

        //OpenCV and webcam initialization
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        pipeline = new EasyOpenCVExample.SkystoneDeterminationPipeline();
        webcam.setPipeline(pipeline);

        //Camera config
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPSIDE_DOWN);
            }
        });

        waitForStart();

        sleep(100);

        int ringAnalysis = pipeline.getAnalysis();

        char SCENARIO = ' ';
        if(ringAnalysis >= fourRingThreshold){
            SCENARIO = 'C';
        }
        else if(ringAnalysis >= oneRingThreshold){
            SCENARIO = 'B';
        }
        else{
            SCENARIO = 'A';
        }

        telemetry.addData("SCENARIO: ", SCENARIO);
        telemetry.update();

        //moveDrivetrain(1,0,0,500);
        moveDrivetrain(0,0.5,0,1750);
        stopDrivetrain();
        sleep(100);
        moveDrivetrain(0,0,-0.25,225);
        launcherLeft.setPower(1);
        launcherRight.setPower(1);

        //angleLauncher(15, 0.5);
        launcherArmL.setPower(1);
        launcherArmR.setPower(1);
        sleep(1325);
        launcherArmL.setPower(0);
        launcherArmR.setPower(0);
        sleep(500);
        pusherServo.setPosition(1);
        sleep(1000);
        launcherRight.setPower(0);
        launcherLeft.setPower(0);
        moveDrivetrain(0,0,0.4,250);

        if(SCENARIO == 'A'){
            moveDrivetrain(0.2,0.1,0,400);
            wobbleArm.setPosition(0);
            sleep(1000);
            wobbleGrip.setPosition(.8);
        }
        if(SCENARIO == 'B'){
            sleep(500);
            moveDrivetrain(-.3,.2,0,2000);
            wobbleArm.setPosition(0);
            sleep(1000);
            wobbleGrip.setPosition(.8);
            sleep(500);
            moveDrivetrain(0,-.1,0,500);
            /**moving to other wobble
            moveDrivetrain(0, -.25, 0, 200);
            moveDrivetrain(0,0,0.5,1900);
            moveDrivetrain(-.1,.5,0,1500);
            sleep(500);
            moveDrivetrain(-.1,.2, 0, 100);
            wobbleGrip.setPosition(0);
            wobbleArm.setPosition(0.4);
             */
        }
        if(SCENARIO == 'C'){
            moveDrivetrain(0,0.5,0,1600);
            sleep(500);
            wobbleArm.setPosition(0);
            sleep(1000);
            wobbleGrip.setPosition(1);
            moveDrivetrain(0,-0.5,0,1200);
        }

        sleep(1000);


        /**
        moveDrivetrain(0.5, 0, -0.15, 250);

        sleep(500);
        pusherServo.setPosition(1);
        sleep(1000);
        launcherLeft.setPower(0);
        launcherRight.setPower(0);
        pusherServo.setPosition(0);

        moveDrivetrain(0.5, 0, 0.17, 275);

        moveDrivetrain(0.75, -0.1, 0.05, 1000);

        sleep(1000);

        if(SCENARIO == 'A'){
            moveDrivetrain(0,0,1,500);
            //Release wobble goal
        }
        else if(SCENARIO == 'B'){
            moveDrivetrain(0,0,-1,500);
            //Release wobble goal
        }
            moveDrivetrain(0,1,0,250);
            moveDrivetrain(0,-1,0,250);
        }
         */

    }

    /**
     * Sets drive train to move in x, y, and rx, for time seconds and then stops
     * @param x
     * @param y
     * @param rx
     * @param time
     */
    public void moveDrivetrain(double x, double y, double rx, int time){
        frontLeft.setPower(y + x + rx);
        rearLeft.setPower(y - x + rx);
        frontRight.setPower(y - x - rx);
        rearRight.setPower(y + x -rx);
        sleep(time);
        stopDrivetrain();
    }

    /**
     * Stops all drive train motors
     */
    public void stopDrivetrain(){
        frontLeft.setPower(0);
        rearLeft.setPower(0);
        frontRight.setPower(0);
        rearRight.setPower(0);
    }

    /**
     * Calculates the rotation of the lift servos required to reach a desired launcher angle
     * @param angle
     */
    public double setLauncherAngle(double angle){
        double a = Math.sqrt(Math.pow(99, 2) + Math.pow(94.4, 2) - (2 * 99 * 94.4 * Math.cos(Math.toRadians(angle + 53.6))));
        double actuatorLength = Math.sqrt((Math.pow(a, 2) - Math.pow(43, 2)));
        double servoAngle = 360 * (actuatorLength - 76) / (36 * Math.PI);
        return servoAngle;
    }

    public void angleLauncher(double angle, double speed){
        double servoAngle = setLauncherAngle(angle);
        launcherArmL.setPower(speed);
        launcherArmR.setPower(speed);
        int sleepTime = (int)(servoAngle / 240 * (1 / speed) * 1000);
        sleep(sleepTime);
    }

    public static class SkystoneDeterminationPipeline extends OpenCvPipeline {

        public enum RingPosition {
            FOUR,
            ONE,
            NONE
        }

        /*
         * Some color constants
         */
        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */
        static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(150, 300);

        static final int REGION_WIDTH = 100;
        static final int REGION_HEIGHT = 100;

        final int FOUR_RING_THRESHOLD = 145;
        final int ONE_RING_THRESHOLD = 135;

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        Mat region1_Cb;
        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;

        private volatile EasyOpenCVExample.SkystoneDeterminationPipeline.RingPosition position = EasyOpenCVExample.SkystoneDeterminationPipeline.RingPosition.FOUR;

        void inputToCb(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame) {
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        }

        @Override
        public Mat processFrame(Mat input) {
            inputToCb(input);

            avg1 = (int) Core.mean(region1_Cb).val[0];

            Imgproc.rectangle(
                    input,
                    region1_pointA,
                    region1_pointB,
                    BLUE,
                    2);

            position = EasyOpenCVExample.SkystoneDeterminationPipeline.RingPosition.FOUR; // Record analysis
            if (avg1 > FOUR_RING_THRESHOLD) {
                position = EasyOpenCVExample.SkystoneDeterminationPipeline.RingPosition.FOUR;
            } else if (avg1 > ONE_RING_THRESHOLD) {
                position = EasyOpenCVExample.SkystoneDeterminationPipeline.RingPosition.ONE;
            } else {
                position = EasyOpenCVExample.SkystoneDeterminationPipeline.RingPosition.NONE;
            }

            Imgproc.rectangle(
                    input,
                    region1_pointA,
                    region1_pointB,
                    GREEN,
                    -1);

            return input;
        }

        public int getAnalysis() {
            return avg1;
        }
    }
}