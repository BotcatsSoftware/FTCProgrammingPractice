package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

// this is the bare bones opmode code if anyone wanted it

//you could put @Autonomous here instead of @TeleOp and it would still work, @TeleOp does not have to go with OpMode
@TeleOp(name = "Example Iterative OpMode", group = "Aditya")
public class AdityaIterative extends OpMode {

    DcMotor topLeft = hardwareMap.dcMotor.get("front_left_motor");
    DcMotor topRight = hardwareMap.dcMotor.get("front_right_motor");
    DcMotor botRight = hardwareMap.dcMotor.get("back_right_motor");
    DcMotor botLeft = hardwareMap.dcMotor.get("back_left_motor");
    //Servo servo1 = hardwareMap.servo.get("servo1");
    //CRServo crServo = hardwareMap.crservo.get("crServo");

    public void init(){


        topLeft.setDirection(DcMotor.Direction.REVERSE);
        botLeft.setDirection(DcMotor.Direction.REVERSE);

        topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        botLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        botRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        botLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        botRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

    }

    public void loop(){
        telemetry.addData("something", "something");
        //telemetry.update();

        topRight.setPower(1);
        topLeft.setPower(1);
        botRight.setPower(1);
        botLeft.setPower(1);

        telemetry.addData("Top Left Encoder", topLeft.getCurrentPosition());
        telemetry.addData("Top Right Encoder", topRight.getCurrentPosition());
        telemetry.addData("Bottom Left Encoder", botLeft.getCurrentPosition());
        telemetry.addData("Bottom Right Encoder", botRight.getCurrentPosition());
        //telemetry.update();

        if(botLeft.getCurrentPosition() > 1000) {
            telemetry.addData("gibberish", "it trued, lets go! (code's in the if statement)");


            //servo1.setPosition(180);

            //if we don't see this and do see the earlier telemetry, we know something went wrong at the servo, if we see this and the servo doesn't move, we know its something with the servo code or hardware
            telemetry.addData("gibberish", "code passed the servo statement");



        }

    }

}
