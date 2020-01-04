package org.firstinspires.ftc.teamcode.Nereus.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Nereus.Autonomous.Init.NereusAggregated;
import org.firstinspires.ftc.teamcode.PID;

import java.util.ArrayList;

@Autonomous(name = "PID Test", group = "Control")
@Disabled
public class PIDTests extends NereusAggregated {
    int window = 2;
    private double speed = 0, out = 0,
            P = 1.5, I = 0, D = 0, epsilon = 1e-4,
            setpoint = 90, error_sum = 0;
    private ArrayList<Double> error_window = new ArrayList<>();
    private PID pid = new PID(P, I, 0, null);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        while(opModeIsActive()) {
            out = adadelta(pid,0.1, out);
            pid.setParams(pid.P - out, I, D, 0.3);
            robot.Left.setPower(speed + out);
            robot.Right.setPower(speed - out);

            telemetry.addData("Angle: ", robot.imu.getAngularOrientation().firstAngle);

            telemetry.addLine()
                    .addData("P", "%f", pid.P)
                    .addData("I", "%f", pid.I)
                    .addData("D", "%f", pid.D);
            telemetry.addData("Power: ", out + speed);
            telemetry.update();
        }
    }

    private double gradient = 0, update = 0, gradient_avg = 0;
    private double adadelta(PID pid, double friction, double update_avg) {
        gradient = pid.getPID(robot.imu.getAngularOrientation().firstAngle);
        gradient_avg = (friction * gradient_avg) + ((1 - friction) * Math.pow(gradient, 2));

        if(update_avg <= 0) {
            update_avg = gradient_avg;
        } else {
            update = (Math.sqrt(Math.pow(update_avg, 2) + epsilon) / Math.sqrt(Math.pow(gradient, 2) + epsilon)) * gradient;
        }

        update_avg = (friction * update_avg) + ((1 - friction) * Math.pow(update, 2));

        telemetry.addLine()
                .addData("Update: ", update)
                .addData("Gradient: ", gradient);
        telemetry.addLine()
                .addData("Update Avg: ", update_avg)
                .addData("Gradient Avg: ", gradient_avg);
        return update_avg;
    }

    private double nesterov(PID pid, double friction, double last) {
        out = (friction * last) + (pid.getPID(robot.imu.getAngularOrientation().firstAngle - ((1 - friction) * last)));
        telemetry.addData("Momentum: ", friction * last);
        return out;
    }

    private void movingwindow(PID pid, int window) {
        error_window.add(pid.error);
        if(error_window.size() > window) {
            error_window.remove(0);
        }

        // Getting sum
        error_sum = 0;
        for(double value : error_window) {
            error_sum += value;
        }

        pid.setParams(P / Math.sqrt(Math.abs(error_sum) + 1), I / Math.sqrt(Math.abs(error_sum) + 1), 0, 0.);
        telemetry.addData("Error Sum: ", error_sum);
    }
}
