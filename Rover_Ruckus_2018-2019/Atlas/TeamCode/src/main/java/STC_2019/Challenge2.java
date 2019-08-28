package STC_2019;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Challenge2", group="STC")
public class Challenge2 extends LinearOpMode {

    private final double countsPerRot = 2240; // The counts per rotation
    private final double drive_gear_reduction = 0.2; // The drive gear reduction of the robot
    private final double wheelDiamInch = 3.54331; // The diameter of the Atlas wheels for finding the circumference
    public double countsPerInch = (countsPerRot * drive_gear_reduction) / (wheelDiamInch * Math.PI);

    public HardwareSTC robot = new HardwareSTC();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();

        encoderDrives(0.4, 130, 130);
        sleep(300);
        encoderDrives(0.4, -11.2, 11.2);
        sleep(300);
        encoderDrives(0.4, 126, 126);
        sleep(300);
        encoderDrives(0.4, -11, 11);
        sleep(300);
        encoderDrives(0.4, 127, 127);
        sleep(300);
        encoderDrives(0.4, -11.5, 11.5);
        sleep(300);
        encoderDrives(0.4, 128, 128);
    }

    public void encoderDrives(double speed,
                              double linches, double rinches) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            robot.Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (robot.Left.isBusy() && robot.Right.isBusy())) {

                // Display it for the driver.
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
