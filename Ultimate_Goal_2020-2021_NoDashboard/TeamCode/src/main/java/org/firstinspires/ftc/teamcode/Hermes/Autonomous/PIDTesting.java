package org.firstinspires.ftc.teamcode.Hermes.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.teamcode.Hermes.Autonomous.Init.HermesAggregated;
import org.firstinspires.ftc.teamcode.Hermes.MiniPID;

@Autonomous(name = "PID Test", group = "Hermes")
public class PIDTesting extends HermesAggregated {

    double delta_angle = 0;
    double previous_angle = 0;
    double total_angle = 0;

    double pid_output;

    public double likeallelse(double angle) {
        delta_angle = angle - previous_angle;

        if (delta_angle < -180)
            delta_angle += 360;
        else if (delta_angle > 180)
            delta_angle -= 360;

        total_angle += delta_angle;
        previous_angle = angle;
        return total_angle;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();

        while(opModeIsActive()) {
            robot.BackLeft.setPower(pid_output);
            robot.FrontLeft.setPower(pid_output);

            robot.BackRight.setPower(-pid_output);
            robot.FrontRight.setPower(-pid_output);
        }



    }
}