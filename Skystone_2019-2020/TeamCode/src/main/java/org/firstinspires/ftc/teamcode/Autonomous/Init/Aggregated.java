package org.firstinspires.ftc.teamcode.Autonomous.Init;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@SuppressWarnings({"unused", "WeakerAccess", "SameParameterValue"})
public class Aggregated extends LinearOpMode {

    public final double countsPerInch = 47.5;
    private double pidOutput = 0;
    public Hardware robot = new Hardware();
    
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
    }

    public void encoderDrives(double speed,
                              double linches, double rinches) {
        int newLeftTarget;
        int newRightTarget;

        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.Left.getCurrentPosition() + (int) (-linches * countsPerInch);
            newRightTarget = robot.Right.getCurrentPosition() + (int) (-rinches * countsPerInch);

            //Both negative cw
            //Both positive backward
            //Left negative cw
            //Right negative backward
            robot.Left.setTargetPosition(newLeftTarget);
            robot.Right.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            robot.Left.setPower(speed);
            robot.Right.setPower(speed);

            robot.Left.setPower(0);
            robot.Right.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void encoderDrives(double speed,
                              double linches, double rinches, PID pid) {
        int newLeftTarget;
        int newRightTarget;

        if (opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.Left.getCurrentPosition() + (int) (-linches * countsPerInch);
            newRightTarget = robot.Right.getCurrentPosition() + (int) (-rinches * countsPerInch);

            //Both negative cw
            //Both positive backward
            //Left negative cw
            //Right negative backward
            robot.Left.setTargetPosition(newLeftTarget);
            robot.Right.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            robot.Left.setPower(speed);
            robot.Right.setPower(speed);

            while (opModeIsActive() &&
                    (robot.Left.isBusy() && robot.Right.isBusy())) {
                pidOutput = pid.getPID(robot.imu.getAngularOrientation().firstAngle);
                robot.Left.setPower(speed - pidOutput);
                robot.Right.setPower(speed + pidOutput);
                // Display it for the driver.
                telemetry.addData("Angle: ", pid.angle);
                telemetry.addData("Error: ", pid.error);
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.Left.getCurrentPosition(),
                        robot.Right.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.Left.setPower(0);
            robot.Right.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
}
