package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.*;

import javax.xml.stream.events.StartElement;

@TeleOp(name = "ArtemDroho")
public class ArtemDroho extends LinearOpMode {
    //declaring motors
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
    String troubleshooting = "d";

    double error = 270, P, I, D, kp = 1, ki = 0, kd = 0.1, integral = 0, derivative, correction, t, lastTime = 0, dt = 0.1, lastError = 90;

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

        topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        //imu init and setup
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        telemetry.addData("INITILIZED: ", topLeft.getClass());
        telemetry.update();
        waitForStart();

        //START CODE
        moveFoward(2000);
        turnAngle(90);

        moveFoward(2000);
        turnAngle(90);

    }


    public void moveFoward(int target){
        int currentPos = 0;

        while(currentPos <= target){
            currentPos = topRight.getCurrentPosition();
            topLeft.setPower(.5);
            topRight.setPower(.5);
            botLeft.setPower(.5);
            botRight.setPower(.5);
        }
        topLeft.setPower(0);
        topRight.setPower(0);
        botLeft.setPower(0);
        botRight.setPower(0);
    }

    private double turnAngle(int angle){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double startTime = System.nanoTime();
        double target = angles.firstAngle + angle;
        //double error = 90, P, I, D, kp = 1, ki = 0, kd = 0.1, integral = 0, derivative, correction, t, lastTime = 0, dt = 0.1, lastError = 90;

        while(error > 1){

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            t = (double)System.nanoTime()/10;
            if(lastTime != 0) {
                dt = t - lastTime;
            }

            error = target - angles.firstAngle;
            integral += (error * dt);
            derivative = (error - lastError)/dt;

            P = kp * error;
            I = ki * integral;
            D = kd * derivative;
            correction = P + I + D;

            topLeft.setPower(-correction);
            topRight.setPower(correction);
            botLeft.setPower(-correction);
            botRight.setPower(correction);

            telemetry.addData("Dt", dt);
            telemetry.addData("Error", error);
            telemetry.addData("top Left Power", topLeft.getPower());
            telemetry.addData("top Right Power", topRight.getPower());
            telemetry.addData("bottom Left Power", botLeft.getPower());
            telemetry.addData("bottom Right Power", botRight.getPower());
            telemetry.addData("Angle", angles.firstAngle);
            telemetry.update();

            lastTime = t;
            lastError = error;
        }

        topLeft.setPower(0);
        topRight.setPower(0);
        botLeft.setPower(0);
        botRight.setPower(0);
        return System.nanoTime() - startTime;
    }
}