package org.firstinspires.ftc.teamcode.Hermes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Hermes.Autonomous.Init.HermesAggregated;
import org.firstinspires.ftc.teamcode.Nereus.Autonomous.Init.NereusAggregated;
import org.firstinspires.ftc.teamcode.PID;

import java.util.ArrayList;

@Autonomous(name = "PID Test", group = "Control")
public class PIDTests extends HermesAggregated {
    int window = 10;
    double speed = 0.1, out = 0,
            P = 3, I = 1, D = 0,
            setpoint = 0, error_sum = 0;
    ArrayList<Double> error_window = new ArrayList<Double>();
    PID pid = new PID(P, I, 0, setpoint);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        while(opModeIsActive()) {
            out = pid.getPID(robot.imu.getAngularOrientation().firstAngle);

            error_window.add(pid.error);
            if(error_window.size() > window) {
                error_window.remove(0);
            }

            // Getting sum
            error_sum = 0;
            for(double value : error_window) {
                error_sum += value;
            }

            pid.setParams(error_sum / P, error_sum / I, 0, setpoint);

            robot.BackLeft.setPower(-speed - out);
            robot.FrontLeft.setPower(-speed - out);
            robot.BackRight.setPower(-speed + out);
            robot.FrontRight.setPower(-speed + out);
            telemetry.addData("Angle: ", robot.imu.getAngularOrientation().firstAngle);
            telemetry.addLine()
                    .addData("P", "%f", pid.P)
                    .addData("I", "%f", pid.I)
                    .addData("D", "%f", pid.D);
            telemetry.addData("Power: ", out + speed);
            telemetry.update();
        }
    }
}
