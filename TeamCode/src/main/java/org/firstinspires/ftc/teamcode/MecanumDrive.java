package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name="mecanumDrive", group="Testing")

public class MecanumDrive extends OpMode {

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

    double frontLeftPower;
    double rearLeftPower;
    double frontRightPower;
    double rearRightPower;
    double launcherLeftPower;
    double launcherRightPower;

    @Override
    public void init() {
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

        pusherServo = hardwareMap.servo.get("pusherServo");
        launcherArmL = hardwareMap.crservo.get("launcherArmL");
        launcherArmR = hardwareMap.crservo.get("launcherArmR");
        wobbleArm = hardwareMap.servo.get("wobbleArm");
        wobbleGrip = hardwareMap.servo.get("wobbleGrip");

        launcherArmL.setDirection(DcMotorSimple.Direction.REVERSE);

        wobbleArm.setPosition(0.4);
        wobbleGrip.setPosition(0);
        pusherServo.setPosition(0);

    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x * 1.5;
        double rx = gamepad1.right_stick_x;
        double launchPower = gamepad1.right_trigger;

        frontLeftPower = (y + x + rx);
        rearLeftPower = (y - x + rx);
        frontRightPower = (y - x - rx);
        rearRightPower = (y + x -rx);


        frontLeft.setPower(frontLeftPower);
        rearLeft.setPower(rearLeftPower);
        frontRight.setPower(frontRightPower);
        rearRight.setPower(rearRightPower);

        launcherLeft.setPower(launchPower);
        launcherRight.setPower(launchPower);

        if(gamepad1.dpad_down){
            pusherServo.setPosition(.6);
        }

        if(gamepad1.dpad_up){
            pusherServo.setPosition(0);
        }

        while(gamepad1.a){
            if(wobbleArm.getPosition() < 1.0){
                wobbleArm.setPosition(wobbleArm.getPosition() + .001);
            }
        }

        while(gamepad1.b){
            if(wobbleArm.getPosition() > 0){
                wobbleArm.setPosition(wobbleArm.getPosition()- .001);
            }
        }

        while(gamepad1.x){
            wobbleGrip.setPosition(1);
        }

        while(gamepad1.y){
            wobbleGrip.setPosition(0);
        }
    }

    @Override
    public void stop() {
        frontLeft.setPower(0);
        rearLeft.setPower(0);
        frontRight.setPower(0);
        rearRight.setPower(0);

        wobbleArm.setPosition(0);
        wobbleGrip.setPosition(0);
    }
}

