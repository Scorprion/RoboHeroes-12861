package Atlas.Calibration;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import Atlas.Autonomous.Init.Aggregated;
import Atlas.Autonomous.Init.PID;

@Autonomous(name = "Control Testing", group = "Calibration")
public class ControlTesting extends Aggregated {
    final double countsPerRot = 2240; // The counts per rotation
    final double gearReductionRatio = 0.5; // The gear box ratio for the motors
    final double wheelDiamInch = 4; // The diameter of the Atlas wheels for finding the circumference
    public final double countsPerInch = 47.5; //(countsPerRot * gearReductionRatio) / (wheelDiamInch * 3.1415);
    private double angle = 0;
    private double pidOutput = 0;
    private double speed = 0.2;

    private DcMotor RightMotor;
    private DcMotor LeftMotor;
    private BNO055IMU imu;

    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    @Override
    public void runOpMode() throws InterruptedException {
        RightMotor = hardwareMap.get(DcMotor.class, "RightMotor");
        LeftMotor = hardwareMap.get(DcMotor.class, "LeftMotor");
        LeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        LeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Very important initialization of the imu config
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);  // REMEMBER THIS TOO

        PID pid = new PID(2.5, 0.5, 0, 90);
        waitForStart();
        //encoderDrives(0.2, 60, 60, pid);
        while(opModeIsActive()) {
            angle = imu.getAngularOrientation().firstAngle;
            pidOutput = pid.getPID(angle);
            LeftMotor.setPower(speed + pidOutput);
            RightMotor.setPower(-speed - pidOutput);
            telemetry.addData("Total Angle: ", pid.total_angle);
            telemetry.addData("Target: ", pid.setpoint);
            telemetry.addData("Current Angle: ", angle);
            telemetry.addData("Output: ", pidOutput);
            telemetry.addData("P: ", pid.Poutput);
            telemetry.addData("I: ", pid.Ioutput);
            telemetry.addData("D: ", pid.Doutput);
            telemetry.addData("Error: ", pid.error);
            telemetry.addData("Parabola", pid.parabola);
            telemetry.update();
        }
    }

    public void encoderDrives(double speed,
                              double linches, double rinches, PID pid) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = LeftMotor.getCurrentPosition() + (int) (-linches * countsPerInch);
            newRightTarget = RightMotor.getCurrentPosition() + (int) (-rinches * countsPerInch);

            //Both negative cw
            //Both positive backward
            //Left negative cw
            //Right negative backward
            LeftMotor.setTargetPosition(newLeftTarget);
            RightMotor.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            LeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            LeftMotor.setPower(speed);
            RightMotor.setPower(speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (LeftMotor.isBusy() && RightMotor.isBusy())) {
                pidOutput = pid.getPID(imu.getAngularOrientation().firstAngle);
                LeftMotor.setPower(speed - pidOutput);
                RightMotor.setPower(speed + pidOutput);
                // Display it for the driver.
                telemetry.addData("Angle: ", pid.angle);
                telemetry.addData("Error: ", pid.error);
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        LeftMotor.getCurrentPosition(),
                        RightMotor.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            LeftMotor.setPower(0);
            RightMotor.setPower(0);

            // Turn off RUN_TO_POSITION
            LeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
}
