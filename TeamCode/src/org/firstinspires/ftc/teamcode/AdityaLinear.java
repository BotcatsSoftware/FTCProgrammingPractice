package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import virtual_robot.controller.VirtualGamePadController;


@TeleOp(name = "Example Linear OpMode", group = "Aditya")
public class AdityaLinear extends LinearOpMode {
    //declaring motors
    //DcMotor intakeLeft;
    //DcMotor intakeRight;
    DcMotor topLeft = hardwareMap.dcMotor.get("front_left_motor");
    DcMotor topRight = hardwareMap.dcMotor.get("front_right_motor");
    DcMotor botRight = hardwareMap.dcMotor.get("back_right_motor");
    DcMotor botLeft = hardwareMap.dcMotor.get("back_left_motor");
    Servo servo1 = hardwareMap.servo.get("back_servo");

    ColorSensor colorSensor = hardwareMap.colorSensor.get("color_sensor");

    DistanceSensor frontDistance = hardwareMap.get(DistanceSensor.class, "front_distance");
    DistanceSensor leftDistance = hardwareMap.get(DistanceSensor.class, "left_distance");
    DistanceSensor rightDistance = hardwareMap.get(DistanceSensor.class, "right_distance");
    DistanceSensor backDistance = hardwareMap.get(DistanceSensor.class, "back_distance");


    BNO055IMU imu = null;

    Orientation angles = null;

    ElapsedTime et = new ElapsedTime();

    boolean flag = true;

    double s = 0.5;

    int t = 0;

    double y;

    double x;


    public void runOpMode() {
        //reversing
        topLeft.setDirection(DcMotor.Direction.REVERSE);
        botLeft.setDirection(DcMotor.Direction.REVERSE);
        //intakeLeft.setDirection(DcMotor.Direction.REVERSE);

        //encoder set up
        topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        botLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        botRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        botLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        botRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //imu init and setup
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        boolean currentState = gamepad1.y;

        telemetry.addData("flag", 1);
        telemetry.update();


        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("flag", 2);
            telemetry.update();

            //real movement code

            //this is wonky cause it won't naturally return to zero like joysticks usually do

            if (gamepad1.left_stick_x < 0.1 && gamepad1.left_stick_x > -0.1 && gamepad1.left_stick_y > -0.1 && gamepad1.left_stick_y < 0.1) {
                telemetry.addData("flag", "left stick stop code");
                telemetry.update();

                topLeft.setPower(0);
                topRight.setPower(0);
                botLeft.setPower(0);
                botRight.setPower(0);
            }

            else {
                telemetry.addData("flag", "vector drive code");
                telemetry.update();

                y = -(gamepad1.left_stick_x * Math.cos(-Math.PI/4) - gamepad1.left_stick_y * Math.sin(-Math.PI/4));
                x = -(gamepad1.left_stick_x * Math.sin(-Math.PI/4) + gamepad1.left_stick_y * Math.cos(-Math.PI/4));
                topLeft.setPower(x);
                topRight.setPower(y);
                botLeft.setPower(y);
                botRight.setPower(x);
            }

            if (gamepad1.right_stick_x > 0.1 || gamepad1.right_stick_x < -0.1){
                topLeft.setPower(gamepad1.right_stick_x);
                topRight.setPower(-gamepad1.right_stick_x);
                botLeft.setPower(gamepad1.right_stick_x);
                botRight.setPower(-gamepad1.right_stick_x);
            }

            else if (gamepad1.left_stick_x < 0.1 && gamepad1.left_stick_x > -0.1) {
                topLeft.setPower(0);
                topRight.setPower(0);
                botLeft.setPower(0);
                botRight.setPower(0);
            }

            /*//arcade movement code
            if(gamepad1.left_stick_y < -0.1 && gamepad1.left_stick_x < 0.3 && gamepad1.left_stick_x > -0.3){
                topLeft.setPower(s * gamepad1.left_stick_y);
                topRight.setPower(s * gamepad1.left_stick_y);
                botLeft.setPower(s * gamepad1.left_stick_y);
                botRight.setPower(s * gamepad1.left_stick_y);
            }

            if(gamepad1.left_stick_y > 0.1 && gamepad1.left_stick_x < 0.3 && gamepad1.left_stick_x > -0.3){
                topLeft.setPower(s * gamepad1.left_stick_y);
                topRight.setPower(s * gamepad1.left_stick_y);
                botLeft.setPower(s * gamepad1.left_stick_y);
                botRight.setPower(s * gamepad1.left_stick_y);
            }

            if(gamepad1.left_stick_x < -0.1 && gamepad1.left_stick_y < 0.3 && gamepad1.left_stick_y > -0.3){
                topLeft.setPower(s * -gamepad1.left_stick_x);
                topRight.setPower(s * gamepad1.left_stick_x);
                botLeft.setPower(s * gamepad1.left_stick_x);
                botRight.setPower(s * -gamepad1.left_stick_x);
            }

            if(gamepad1.left_stick_x > 0.1 && gamepad1.left_stick_y < 0.3 && gamepad1.left_stick_y > -0.3){
                topLeft.setPower(s * gamepad1.left_stick_x);
                topRight.setPower(s * -gamepad1.left_stick_x);
                botLeft.setPower(s * -gamepad1.left_stick_x);
                botRight.setPower(s * gamepad1.left_stick_x);
            }*/


            //y button toggle for changing motor speeds (fast mode and slow mode)
            if (gamepad1.dpad_down){
                servo1.setPosition(180);
            }

            if (gamepad1.y && t == 0 && flag) {
                s = 1;
                t++;
                flag = false;
            }
            else if (gamepad1.y && t == 1 && flag) {
                s = 0.25;
                t++;
                flag = false;
            }
            else if (gamepad1.y && t == 2 && flag) {
                s = 0.5;
                t = 0;
                flag = false;
            }
            else if (!gamepad1.y && !flag) {
                flag = true;
            }

            //running hypothetical intake motors on triggers
            /*if (gamepad1.right_trigger > 1){ //what are the values?
                intakeLeft.setPower(1);
                intakeRight.setPower(1);
            }

            else if (gamepad1.left_trigger > 1){ //what are the values?
                intakeLeft.setPower(-1);
                intakeRight.setPower(-1);
            }*/




        }


    }


}
