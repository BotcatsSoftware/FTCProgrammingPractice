package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


// this is the bare bones LinearOpMode code if anyone wanted it

//you could put @TeleOp here instead of @Autonomous and it would still work, @Autonomous does not have to go with OpMode

@TeleOp(name = "Previous Assignment", group = "Aditya")
public class AdityaPreviousAssignment extends LinearOpMode{
    //declaring motors
    DcMotor topLeft = hardwareMap.dcMotor.get("front_left_motor");
    DcMotor topRight = hardwareMap.dcMotor.get("front_right_motor");
    DcMotor botRight = hardwareMap.dcMotor.get("back_right_motor");
    DcMotor botLeft = hardwareMap.dcMotor.get("back_left_motor");
    Servo servo1 = hardwareMap.servo.get("back_servo");


    ElapsedTime et = new ElapsedTime();


    //CRServo crServo = hardwareMap.crservo.get("crServo");


    public void runOpMode() {
        //reversing
        topLeft.setDirection(DcMotor.Direction.REVERSE);
        botLeft.setDirection(DcMotor.Direction.REVERSE);

        //encoder set up
        topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        botLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        botRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        botLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        botRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //imu init and setup
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;

        imu.initialize(parameters);

        int i = 180;
        int enc;

        waitForStart();
        while (opModeIsActive()) {

            //forward
            et.reset();
        while(et.milliseconds() < 2000) {
            topRight.setPower(1);
            topLeft.setPower(1);
            botRight.setPower(1);
            botLeft.setPower(1);
        }

        //backwards
        et.reset();
        while(et.milliseconds() < 2000) {
            topRight.setPower(-1);
            topLeft.setPower(-1);
            botRight.setPower(-1);
            botLeft.setPower(-1);
        }

        //left
        et.reset();
        while(et.milliseconds() < 2000) {
            topRight.setPower(1);
            topLeft.setPower(-1);
            botRight.setPower(-1);
            botLeft.setPower(1);
        }

        //right
        et.reset();
        while(et.milliseconds() < 2000) {
            topRight.setPower(-1);
            topLeft.setPower(1);
            botRight.setPower(1);
            botLeft.setPower(-1);
        }

        //spin counter-clockwise
        et.reset();
        while(et.milliseconds() < 2000) {
            topRight.setPower(1);
            topLeft.setPower(-1);
            botRight.setPower(1);
            botLeft.setPower(-1);
        }

        //spin clockwise
        et.reset();
        while(et.milliseconds() < 2000) {
            topRight.setPower(-1);
            topLeft.setPower(1);
            botRight.setPower(-1);
            botLeft.setPower(1);
        }


        servo1.setPosition(i);
        i *= -1;


            enc = topRight.getCurrentPosition();

            while(topRight.getCurrentPosition() < enc + 5000) {
                telemetry.addData("code location", "first while");
                telemetry.update();

                topRight.setPower(1);
                topLeft.setPower(1);
                botRight.setPower(1);
                botLeft.setPower(1);
            }

            while(topRight.getCurrentPosition() > 0){
                telemetry.addData("code location", "second while");
                telemetry.update();

                topRight.setPower(-1);
                topLeft.setPower(-1);
                botRight.setPower(-1);
                botLeft.setPower(-1);
            }

            while(topRight.getCurrentPosition() < 5000){
                telemetry.addData("code location", "third while");
                telemetry.update();

                topRight.setPower(1);
                topLeft.setPower(-1);
                botRight.setPower(-1);
                botLeft.setPower(1);
            }

            while(topRight.getCurrentPosition() > 0){
                telemetry.addData("code location", "fourth while");
                telemetry.update();

                topRight.setPower(-1);
                topLeft.setPower(1);
                botRight.setPower(1);
                botLeft.setPower(-1);
            }

            while (topRight.getCurrentPosition() < 5000){
                telemetry.addData("code location", "first rotate while");
                telemetry.update();

                topRight.setPower(1);
                topLeft.setPower(-1);
                botRight.setPower(1);
                botLeft.setPower(-1);
            }

            while (topRight.getCurrentPosition() > 0){
                telemetry.addData("code location", "second rotate while");
                telemetry.update();

                topRight.setPower(-1);
                topLeft.setPower(1);
                botRight.setPower(-1);
                botLeft.setPower(1);
            }


            //servo stuf
            telemetry.addData("code position", "before the servo");
            telemetry.update();

            i *= -1;

            servo1.setPosition(i);

            if(servo1.getPosition() == 180){
                telemetry.addData("servo status", "set to 180");
                telemetry.update();
            }

            if(servo1.getPosition() == -180){
                telemetry.addData("servo status", "set to 180");
                telemetry.update();
            }

            telemetry.addData("code position", "past the servo");
            telemetry.update();


        }

    }
}
