package Atlas.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import Atlas.Autonomous.Init.Aggregated;

@Autonomous(name = "AtlasAuto", group = "Atlas")
public class AtlasAuto extends Aggregated {

    final double countsPerRot = 2240; // The counts per rotation
    final double gearBoxRatio = 0.5; // The gear box ratio for the motors
    final double wheelDiamInch = 4; // The diameter of the Atlas wheels for finding the circumference
    public final double countsPerInch = (countsPerRot * gearBoxRatio) / (wheelDiamInch * 3.1415);
    private double output = 1; // Needs to be 100 to get passed the "while" statement's conditions
    private double angle = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

            // Display it for the driver.
            telemetry.addData("Path2",  "Running at %7d :%7d",
                    robot.Left.getCurrentPosition(),
                    robot.Right.getCurrentPosition());
            telemetry.update();
        }
        /*PID(0.7, 0.1, 1, 90);
        while(opModeIsActive()) {
            angle = eulerNormalize(robot.imu.getAngularOrientation().firstAngle);
            telemetry.addData("Current Angle: ", angle);
            telemetry.update();
        }/*
        encoderDrives(0.4, 5, 5);

        PID(1.3, 0.01, 0.7, 88);

        encoderDrives(0.4, 10, 10);
        sleep(100000);*/
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.Left.getCurrentPosition() + (int)(-leftInches * countsPerInch);
            newRightTarget = robot.Right.getCurrentPosition() + (int)(-rightInches * countsPerInch);
            robot.Left.setTargetPosition(newLeftTarget);
            robot.Right.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            robot.Left.setPower(speed);
            robot.Right.setPower(speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (robot.Left.isBusy() && robot.Right.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
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
