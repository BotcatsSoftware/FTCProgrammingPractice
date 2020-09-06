package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Limberg_PID {
    @Autonomous(name = "Limberg_PID", group = "RedLine")
    public class Limberg_PIDgo  extends LinearOpMode {
        public void runOpMode() {

        }
        public void turn90Degrees() {
            /*
            angle = imu.getAnglularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double target = angles.firstAngle+90;
            double error= 90, P, I, D, kp = 0, kd = 0.1; integral = 0, derivative, correction, t, lastTime = 0, dt = 0.1, lastError = 90;
            while (error>3) {
                angle = imu.getAnglularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                t = et.nanoseconds / 10;
                dt = t-lastTime;
                error = target - angles.firstAngle;
                integral += (error*dt);
                derivative = (error-lastError) / dt;
                P = kp * errpr;
                I = ki * integral;
                D = kd * derivative;
                correction = P + I + D;
                Right.setPower(correction);
                Left.setPower(-correction);
                telemetry.addData (caption: "dt", dt);
                telemetry.addData (caption: "error", error);
                telemetry.addData (caption: "correction", correction);
                lastTime = t;
                lastError = error;

            }
            */

        }
    }
}
