package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous(name = "Eli's robot code")
public class EliPregerson extends LinearOpMode {
    DcMotor fl;
    DcMotor fr;
    DcMotor bl;
    DcMotor br;
    Orientation angles;
    BNO055IMU imu;
    public void runOpMode(){
        fl = hardwareMap.dcMotor.get("front_left_motor");
        fr = hardwareMap.dcMotor.get("front_right_motor");
        bl = hardwareMap.dcMotor.get("back_left_motor");
        br = hardwareMap.dcMotor.get("back_right_motor");

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        Servo servo = hardwareMap.servo.get("back_servo");



        DistanceSensor frontDistance = hardwareMap.get(DistanceSensor.class, "front_distance");
        DistanceSensor leftDistance = hardwareMap.get(DistanceSensor.class, "left_distance");
        DistanceSensor rightDistance = hardwareMap.get(DistanceSensor.class, "right_distance");
        DistanceSensor backDistance = hardwareMap.get(DistanceSensor.class, "back_distance");


        ColorSensor colorSensor = hardwareMap.colorSensor.get("color_sensor");


        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);




        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart();
        telemetry.addData("How long does it take to turn", turn90Degrees());
        telemetry.update();
    }





    private double turn90Degrees(){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double startTime = System.nanoTime();
        double target = angles.firstAngle + 90;
        double error = 90, P, I, D, kp = .3, ki = 0, kd = .4, integral = 0, derivative, correction, t, lt = 0, dt = 0.1, lastError = 90;

        while(error > .1){

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            t = (double)System.nanoTime()/10;
            if(lt != 0) {
                dt = t - lt;
            }

            error = target - angles.firstAngle;
            integral += (error * dt);
            derivative = (error - lastError)/dt;

            P = kp * error;
            I = ki * integral;
            D = kd * derivative;
            correction = P + I + D;

            fl.setPower(-correction);
            fr.setPower(correction);
            bl.setPower(-correction);
            br.setPower(correction);

            telemetry.addData("Dt", dt);
            telemetry.addData("Error", error);
            telemetry.addData("Fl Power", fl.getPower());
            telemetry.addData("Fr Power", fr.getPower());
            telemetry.addData("Bl Power", bl.getPower());
            telemetry.addData("Br Power", br.getPower());
            telemetry.addData("Angle", angles.firstAngle);
            telemetry.update();

            lt = t;
            lastError = error;
        }

        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
        return System.nanoTime() - startTime;
    }
}
