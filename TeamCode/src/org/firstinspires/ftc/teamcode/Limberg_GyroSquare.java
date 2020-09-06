package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.*;

@Autonomous(name = "Limberg_GyroSquare", group = "Redline")
public class Limberg_GyroSquare extends LinearOpMode{
    public void runOpMode() {
        // Declarations Begin
        //Motors

        DcMotor left = hardwareMap.dcMotor.get("front_left_motor");
        DcMotor right = hardwareMap.dcMotor.get("front_right_motor");
        DcMotor back_left = hardwareMap.dcMotor.get("back_left_motor");
        DcMotor back_right = hardwareMap.dcMotor.get("back_right_motor");
        Servo servo1 = hardwareMap.servo.get("back_servo");
        left.setDirection(DcMotor.Direction.REVERSE);
        right.setDirection(DcMotor.Direction.FORWARD);
        back_left.setDirection(DcMotor.Direction.REVERSE);
        back_right.setDirection(DcMotor.Direction.FORWARD);
        //Sensors
        ColorSensor colorSensor = hardwareMap.colorSensor.get("color_sensor");
        DistanceSensor frontDistance = hardwareMap.get(DistanceSensor.class, "front_distance");
        DistanceSensor leftDistance = hardwareMap.get(DistanceSensor.class, "left_distance");
        DistanceSensor rightDistance = hardwareMap.get(DistanceSensor.class, "right_distance");
        DistanceSensor backDistance = hardwareMap.get(DistanceSensor.class, "back_distance");
        BNO055IMU imu = null;
        Orientation angles = null;
        ElapsedTime et = new ElapsedTime();


        // Gyro
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
        //Telemetry
        telemetry.addData("Gyro Reading", angles.firstAngle);
        telemetry.addData("frontDistance", frontDistance.getDistance(DistanceUnit.CM));
        telemetry.addData("leftDistance", leftDistance.getDistance(DistanceUnit.CM));
        telemetry.addData("rightDistance", rightDistance.getDistance(DistanceUnit.CM));
        telemetry.addData("backDistance", backDistance.getDistance(DistanceUnit.CM));

        telemetry.update();
        //Variables
        int c = 0;
        waitForStart();



        while (opModeIsActive()) {
            //Declarations end


            //Program Body Begin
            while (c < 4) {
                int enc = right.getCurrentPosition();
                telemetry.addData("Encoder", enc);
                while (right.getCurrentPosition() < enc + 5000) {
                    telemetry.addData("flag", 1);
                    telemetry.update();
                    right.setPower(1);
                    left.setPower(1);
                    telemetry.update();
                    if (right.getCurrentPosition() >= enc + 5000) {
                        telemetry.addData("flag", 2);
                        telemetry.update();
                        right.setPower(0);
                        left.setPower(0);
                turn90Degrees();
                        c++;
                    }


                }
            }
        }

    }
    public void turn90Degrees() {

        // Declarations Begin
        //Motors

        DcMotor left = hardwareMap.dcMotor.get("front_left_motor");
        DcMotor right = hardwareMap.dcMotor.get("front_right_motor");
        DcMotor back_left = hardwareMap.dcMotor.get("back_left_motor");
        DcMotor back_right = hardwareMap.dcMotor.get("back_right_motor");
        Servo servo1 = hardwareMap.servo.get("back_servo");
        left.setDirection(DcMotor.Direction.REVERSE);
        right.setDirection(DcMotor.Direction.FORWARD);
        back_left.setDirection(DcMotor.Direction.REVERSE);
        back_right.setDirection(DcMotor.Direction.FORWARD);
        //Sensors
        ColorSensor colorSensor = hardwareMap.colorSensor.get("color_sensor");
        DistanceSensor frontDistance = hardwareMap.get(DistanceSensor.class, "front_distance");
        DistanceSensor leftDistance = hardwareMap.get(DistanceSensor.class, "left_distance");
        DistanceSensor rightDistance = hardwareMap.get(DistanceSensor.class, "right_distance");
        DistanceSensor backDistance = hardwareMap.get(DistanceSensor.class, "back_distance");
        BNO055IMU imu = null;
        Orientation angles = null;
        ElapsedTime et = new ElapsedTime();
        int enc = right.getCurrentPosition();

        // Gyro
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
        //Telemetry
        telemetry.addData("Gyro Reading", angles.firstAngle);
        telemetry.addData("frontDistance", frontDistance.getDistance(DistanceUnit.CM));
        telemetry.addData("leftDistance", leftDistance.getDistance(DistanceUnit.CM));
        telemetry.addData("rightDistance", rightDistance.getDistance(DistanceUnit.CM));
        telemetry.addData("backDistance", backDistance.getDistance(DistanceUnit.CM));
        telemetry.addData("Encoder", enc);
        telemetry.update();



            double target = angles.firstAngle+90;
            double error= 90, P, I, D, kp = 0.4, kd = 0.1, ki = 0, integral = 0, derivative, correction, t, lastTime = 0, dt = 0.1, lastError = 90;
            while (error>3) {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                t = (double)System.nanoTime() / 10;
                dt = t - lastTime;
                error = target - angles.firstAngle;
                integral += (error * dt);
                derivative = (error - lastError) / dt;
                P = kp * error;
                I = ki * integral;
                D = kd * derivative;
                correction = P + I + D;
                right.setPower(correction);
                left.setPower(-correction);
                back_right.setPower(correction);
                back_left.setPower(-correction);
                telemetry.addData("dt", dt);
                telemetry.addData("error", error);
                telemetry.addData("correction", correction);
                telemetry.addData("angle", angles.firstAngle);
                telemetry.update();
                lastTime = t;
                lastError = error;
            }
        right.setPower(0);
        left.setPower(0);
        back_right.setPower(0);
        back_left.setPower(0);



            }

}
