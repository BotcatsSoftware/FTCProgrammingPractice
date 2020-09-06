package org.firstinspires.ftc.teamcode.ftc16072;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.*;

@Autonomous(name = "Mravac PID")
public class MravacPID extends LinearOpMode {

    ElapsedTime et = new ElapsedTime();
    DcMotor topLeft = hardwareMap.dcMotor.get("front_left_motor");
    DcMotor topRight = hardwareMap.dcMotor.get("front_right_motor");
    DcMotor botLeft = hardwareMap.dcMotor.get("back_left_motor");
    DcMotor botRight = hardwareMap.dcMotor.get("back_right_motor");

    DistanceSensor distSens = hardwareMap.get(DistanceSensor.class, "front");

    BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    Orientation angles = null;

    @Override
    public void runOpMode() throws InterruptedException {
        topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        topLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        botLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        imu.initialize(parameters);
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

        waitForStart();


        for(int x = 0; x < 4; x++){
            double enc = topLeft.getCurrentPosition();
            move:
            while(topLeft.getCurrentPosition() < enc + 5000){
                topLeft.setPower(1);
                topRight.setPower(1);
                botLeft.setPower(1);
                botRight.setPower(1);
                if(distSens.getDistance(DistanceUnit.CM) <= 50) {
                    break move;
                }
            }
            turnDegrees(90);
        }


    }
    double error, P, I, D, lastTime, t, dt, lastError, integral, derivative, kp = 0.3, ki = 0.03, kd = 0.4, correction;

    public void turnDegrees(int angle){
        error = angle;
        t = et.seconds();
        if(lastTime == 0){
            dt = 0.1;
        }
        else{
            dt = t - lastTime;
        }
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double setPoint = angles.firstAngle + angle;
        integral += (error * dt);
        derivative = (error - lastError)/ dt;

        P = kp * error;
        I = ki * integral;
        D = kd * derivative;

        if(error < 3 && error > -3){
            correction = P + I + D;
        }
        else {
            correction = P + D;
        }

        topLeft.setPower(correction);
        botLeft.setPower(correction);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }
}
