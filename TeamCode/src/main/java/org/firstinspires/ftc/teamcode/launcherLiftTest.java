package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import static android.os.SystemClock.sleep;

@TeleOp(name="launcherTest", group="Testing")

public class launcherLiftTest extends OpMode {

    CRServo launcherArmL;
    CRServo launcherArmR;
    DigitalChannel homeSwitch;

    @Override
    public void init() {
        launcherArmL = hardwareMap.crservo.get("launcherArmL");
        launcherArmR = hardwareMap.crservo.get("launcherArmR");
        homeSwitch = hardwareMap.get(DigitalChannel.class, "homeSwitch");
        launcherArmL.setDirection(DcMotorSimple.Direction.REVERSE);

        //Home the launcher
        double launcherArmPosition;

        while (homeSwitch.getState() == true){
            launcherArmL.setPower(-0.25);
            launcherArmR.setPower(-0.25);

        }

        launcherArmL.setPower(0);
        launcherArmR.setPower(0);

        launcherArmPosition = 0.0;
    }

    @Override
    public void loop() {
        if(gamepad1.a){
            launcherArmL.setPower(1);
            launcherArmR.setPower(1);
        }
        else if(gamepad1.b && homeSwitch.getState()){
            launcherArmL.setPower(-1);
            launcherArmR.setPower(-1);
        }
        else{
            launcherArmL.setPower(0);
            launcherArmR.setPower(0);
        }


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
}
