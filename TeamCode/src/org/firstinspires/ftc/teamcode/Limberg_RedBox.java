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

@Autonomous(name = "Limberg_RedBox", group = "RedLine")
public class Limberg_RedBox extends LinearOpMode {
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
        //Variables
        int c = 0;
        //Declarations End

        //Program Body Begin


        while (right.getCurrentPosition() > enc - 500 && c == 0) {
            right.setPower(-1);
            back_right.setPower(1);
            left.setPower(1);
            back_left.setPower(-1);
            if (right.getCurrentPosition() < enc - 500) {
                right.setPower(0);
                back_right.setPower(0);
                left.setPower(0);
                back_left.setPower(0);
                c = 1;


            }
        }






            while (colorSensor.red() >= 100) {
                left.setPower(1);
                right.setPower(-1);
                back_left.setPower(1);
                back_right.setPower(-1);
                telemetry.addData("frontDistance", frontDistance.getDistance(DistanceUnit.CM));
                telemetry.addData("leftDistance", leftDistance.getDistance(DistanceUnit.CM));
                telemetry.addData("rightDistance", rightDistance.getDistance(DistanceUnit.CM));
                telemetry.addData("backDistance", backDistance.getDistance(DistanceUnit.CM));
                telemetry.addData("Encoder", enc);
                telemetry.addData("Gyro Reading", angles.firstAngle);
                String thirdLoop = "Yes";
                telemetry.addData("third loop", thirdLoop);
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
                telemetry.update();


            }



    }
}