package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.*;

import static java.lang.Math.cos;
// this is the bare bones LinearOpMode code if anyone wanted it
//you could put @TeleOp here instead of @Autonomous and it would still work, @Autonomous does not have to go with OpMode

@TeleOp(name = "My_Code", group = "Rohan_S")
public class Rohan_S extends LinearOpMode {
    //Motors:
    DcMotor tl = hardwareMap.dcMotor.get("front_left_motor");
    DcMotor tr = hardwareMap.dcMotor.get("front_right_motor");
    DcMotor br = hardwareMap.dcMotor.get("back_right_motor");
    DcMotor bl = hardwareMap.dcMotor.get("back_left_motor");


    //Servo
    Servo servo1 = hardwareMap.servo.get("back_servo");

    //Distance sensors
    DistanceSensor frontDistance = hardwareMap.get(DistanceSensor.class, "front_distance");
    DistanceSensor rightDistance = hardwareMap.get(DistanceSensor.class, "right_distance");
    DistanceSensor leftDistance = hardwareMap.get(DistanceSensor.class, "left_distance");
    DistanceSensor backDistance = hardwareMap.get(DistanceSensor.class, "back_distance");
    //Function example: frontDistance.getDistance(DistanceUnit.CM)

    //Color sensor
    ColorSensor colorSensor = hardwareMap.colorSensor.get("color_sensor");
    //For red line, color sensor value for red is 174.
    //For blue line, color sensor value for red is 52.

    //Time function (make sure to reset it every time you want to use it)
    ElapsedTime et = new ElapsedTime();
    //CRServo servo2 = hardwareMap.crservo.get("servo2");

    public void setupAll(){
        //because you don't wanna keep setting directions over and over again
        //it's easier simply to code all motor action set in the same direction
        tl.setDirection(DcMotor.Direction.FORWARD);
        bl.setDirection(DcMotor.Direction.FORWARD);
        tr.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.REVERSE);
        //Encoders:
        tr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        tr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }
    public void resetPower(){
        tl.setPower(0);
        tr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
    }
    public void goForward(double a){
        //a is the power multiplier
        tl.setPower(-1 * a);
        tr.setPower(-1 * a);
        bl.setPower(-1 * a);
        br.setPower(-1 * a);
    }
    public void goBackward(double a){
        //a is the power multiplier
        tl.setPower(1 * a);
        tr.setPower(1 * a);
        bl.setPower(1 * a);
        br.setPower(1 * a);
    }
    public void strafe(String a, double b){
        //b is the power multiplier
        //a is whether to go left or right
        if (a.equals("r") || a.equals("right")){
            tl.setPower(-1 * b);
            tr.setPower(1 * b);
            bl.setPower(1 * b);
            br.setPower(-1 * b);
        }
        else{
            tl.setPower(1 * b);
            tr.setPower(-1 * b);
            bl.setPower(-1 * b);
            br.setPower(1 * b);
        }
    }
    public void spin(String a, double b){
        if (a.equals("cw")){
            tl.setPower(1 * b);
            tr.setPower(-1 * b);
            bl.setPower(1 * b);
            br.setPower(-1 * b);
        }
        else{
            tl.setPower(-1 * b);
            tr.setPower(1 * b);
            bl.setPower(-1 * b);
            br.setPower(1 * b);
        }
    }
    public void resetPosition(){
        if (leftDistance.getDistance(DistanceUnit.CM) < 820){
            while(leftDistance.getDistance(DistanceUnit.CM) < 820)
                strafe("right", 1);
        }
        else if(leftDistance.getDistance(DistanceUnit.CM) == 820){
            et.reset();
            while(et.seconds() < 0.01)
                resetPower();
        }
        else{
            while(leftDistance.getDistance(DistanceUnit.CM) > 820)
                strafe("left", 1);
        }
        if (frontDistance.getDistance(DistanceUnit.CM) < 820){
            while(frontDistance.getDistance(DistanceUnit.CM) < 820)
                goBackward(1);
        }
        else if (frontDistance.getDistance(DistanceUnit.CM) == 820){
            et.reset();
            while(et.seconds() < 0.01)
                resetPower();
        }
        else{
            while(frontDistance.getDistance(DistanceUnit.CM) > 820)
                goForward(1);
        }
    }
    public void square(){
        //Setting position to bottom left corner
        while(leftDistance.getDistance(DistanceUnit.CM) > 10)
            strafe("l", 1);
        while(backDistance.getDistance(DistanceUnit.CM) > 10)
            goBackward(1);
        et.reset();
        while(et.seconds() < 2)
            resetPower();
        //Actually moving in a square
        while(frontDistance.getDistance(DistanceUnit.CM) > 10)
            goForward(1);
        while(rightDistance.getDistance(DistanceUnit.CM) > 10)
            strafe("r", 1);
        while(backDistance.getDistance(DistanceUnit.CM) > 10)
            goBackward(1);
        while(leftDistance.getDistance(DistanceUnit.CM) > 10)
            strafe("l", 1);
    }
/*
    public void turnDegrees(int angle){
        Orientation angles =
        double target = angles.firstAngle + angle;
        double error = 90, P, I, D, kp = 0.1, ki = 0, kd = 0.1, integral = 0, derivative, correction, t, lastTime = 0, dt = 0.1, lastError = 90;

        while (error > 3){
            BNO055IMU imu = null;
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            t = et.nanoseconds() / 10;
            if (lastTime != 0){
                dt = t - lastTime;
            }

            error = target - angles.firstAngle;
            integral += (error * dt);
            derivative = (error - lastError) / dt;

            P = kp * error;
            I = ki * integral;
            D = kd * derivative;
            correction = P + I + D;

            tr.setPower(correction);
            tl.setPower(-correction);
            br.setPower(correction);
            bl.setPower(-correction);

            telemetry.addData("dt", dt);
            telemetry.addData("error", error);
            telemetry.addData("correction", correction);
            telemetry.update();

            lastTime = t;
            lastError = error;
        }
    }
*/
    public void pidSquare() {
        double i = -1 * tr.getCurrentPosition();
        while (i < 5000) {
            goForward(1);
            i = -1 * tr.getCurrentPosition();
        }
    }
    public void test(int t){
        et.reset();
        while(et.seconds() < t) {
            double y = -(gamepad1.left_stick_x * Math.cos(-Math.PI / 4) - gamepad1.left_stick_y * Math.sin(-Math.PI / 4));
            double x = -(gamepad1.left_stick_x * Math.sin(-Math.PI / 4) + gamepad1.left_stick_y * Math.cos(-Math.PI / 4));
            int p = 4;
            tl.setPower(p * x);
            tr.setPower(-1 * p * y);
            bl.setPower(-1 * p * y);
            br.setPower(p * x);
        }
    }


    public void runOpMode() {
        //imu (gyro) init and setup
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        Orientation angles = null;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double i = cos(45);

        setupAll();
        resetPower();
        resetPosition();
        test(1000);
        //pidSquare();
        //square();
        //turnDegrees(90);
        //resetPosition();
    }
}