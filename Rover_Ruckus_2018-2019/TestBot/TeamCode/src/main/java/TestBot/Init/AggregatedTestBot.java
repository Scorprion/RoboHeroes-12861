package TestBot.Init;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class AggregatedTestBot extends LinearOpMode {
    /**
     * A program that houses all of our methods needed to run our robot's
     * autonomous programs
     * <p>
     * Since we couldn't use interfaces or anything like that to be able to implement different set
     * methods like
     *
     * @param encoderDrive
     */

    //Using our robot's hardware
    HardwareTestBot robot = new HardwareTestBot();

    //Defining final variables for the encoders
    final double countsPerRot = 2240; // The counts per rotation
    final double gearBoxRatio = 0.5; // The gear box ratio for the motors
    final double wheelDiamInch = 4; // The diameter of the Atlas wheels for finding the circumference
    final double countsPerInch = (countsPerRot * gearBoxRatio) / (wheelDiamInch * 3.1415);

    @Override
    public void runOpMode() throws InterruptedException {
        //Initializing
        robot.init(hardwareMap);
    }

    /**
     * Encoder method for controlling our autonomous programs
     *
     * @param speed   The speed at which the motors turn at
     * @param linches the distance for the left motor to turn (in inches)
     * @param rinches the distance for the right motor to turn (in inches)
     */
    public void encoderDrives(double speed,
                             double linches, double rinches) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.Left.getCurrentPosition() + (int)(linches * countsPerInch);
            newRightTarget = robot.Right.getCurrentPosition() + (int)(rinches * countsPerInch);
            robot.Left.setTargetPosition(newLeftTarget);
            robot.Right.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            robot.Left.setPower(-Math.abs(speed));
            robot.Right.setPower(Math.abs(speed));

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

    /*public double getError(double tangle, double cangle) {
        double robotError = 0;

        if(cangle > 180) {
            robotError = cangle + tangle;
        }
        if(cangle <= -180) {
            robotError = cangle - tangle;
        }

        return robotError;
    }*/
}
