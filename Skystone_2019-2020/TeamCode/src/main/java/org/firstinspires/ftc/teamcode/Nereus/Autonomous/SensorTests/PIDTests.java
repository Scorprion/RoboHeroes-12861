package org.firstinspires.ftc.teamcode.Nereus.Autonomous.SensorTests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Nereus.Autonomous.Init.Aggregated;
import org.firstinspires.ftc.teamcode.PID;

@Autonomous(name = "PID Test", group = "Control")
public class PIDTests extends Aggregated {
    double speed = 0.2, leftout, rightout;
    PID leftpid = new PID(0.5, 0.5, 0, 90);
    PID rightpid = new PID(0.5, 0.3, 0, 90);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);


        while(opModeIsActive()) {
            leftout = leftpid.getPID(robot.imu.getAngularOrientation().firstAngle);
            rightout = rightpid.getPID(robot.imu.getAngularOrientation().firstAngle);
            robot.Left.setPower(speed + leftout);
            robot.Right.setPower(-speed + rightout);
            telemetry.addData("Angle: ", robot.imu.getAngularOrientation().firstAngle);
            telemetry.addData("Left power: ", leftout + speed);
            telemetry.addData("Right power: ", -rightout + speed);
            telemetry.update();
        }
    }
}
