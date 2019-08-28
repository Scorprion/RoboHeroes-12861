package STC_2019;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;
import STC_2019.Challenge2;

@Autonomous(name="Challenge_2_2", group="STC")
public class Challenge2_2 extends LinearOpMode {
    private NormalizedRGBA colors = new NormalizedRGBA();
    private NormalizedRGBA colors2 = new NormalizedRGBA();
    private float[] hsvValues = new float[3];
    private int color = 0;
    private int color2 = 0;
    //public static final double turnSpeed = 0.5;
    public boolean colorFound = false;
    private boolean markerFound = false;
    private double diffred = 0, diffblue = 0;
    private final double countsPerRot = 2240; // The counts per rotation
    private final double drive_gear_reduction = 0.2; // The drive gear reduction of the robot
    private final double wheelDiamInch = 3.54331; // The diameter of the Atlas wheels for finding the circumference
    public double countsPerInch = (countsPerRot * drive_gear_reduction) / (wheelDiamInch * Math.PI);

    public HardwareSTC robot = new HardwareSTC();

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();

        robot.runtime.reset();
        while (opModeIsActive()) {
            while (opModeIsActive() && !colorFound && (robot.runtime.milliseconds() < 500)) {
                colors = robot.Color1.getNormalizedColors();
                color = colors.toColor();
                colors.red /= 255;
                colors.green /= 255;
                colors.blue /= 255;
                color = colors.toColor();

                colors2 = robot.Color2.getNormalizedColors();
                color2 = colors.toColor();
                colors2.red /= 255;
                colors2.green /= 255;
                colors2.blue /= 255;
                color2 = colors.toColor();

                diffred = 0; //defaultRed - colors.red;
                diffblue = 0; //defaultBlue - colors.blue;

                telemetry.addData("Red:", colors.red);
                telemetry.addData("Calibrated Red:", diffred);
                telemetry.addData("Blue:", colors.blue + diffblue);
                telemetry.addData("Calibrated Blue:", diffblue);
                telemetry.update();
                if ((Color.red(color) <= 50 && Color.blue(color) <= 50 && Color.green(color) <= 50)||(Color.red(color2) <= 50 && Color.blue(color2) <= 50 && Color.green(color2) <= 50)) {
                    encoderDrives(0.4, 130, 126);
                    sleep(300);

                }
            }
        }
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
