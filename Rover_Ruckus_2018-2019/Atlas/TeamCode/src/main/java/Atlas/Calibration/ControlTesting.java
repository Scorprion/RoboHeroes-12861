package Atlas.Calibration;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import Atlas.Autonomous.Init.Aggregated;
import Atlas.Autonomous.Init.PID;

@Autonomous(name = "Control Testing", group = "Atlas")
public class ControlTesting extends Aggregated {
    final double countsPerRot = 2240; // The counts per rotation
    final double gearReductionRatio = 0.5; // The gear box ratio for the motors
    final double wheelDiamInch = 4; // The diameter of the Atlas wheels for finding the circumference
    public final double countsPerInch = (countsPerRot * gearReductionRatio) / (wheelDiamInch * 3.1415);
    private double angle = 0;
    private double pidOutput = 0;
    private double speed = 0.1;

    private DcMotor RightMotor;
    private DcMotor LeftMotor;
    private BNO055IMU imu;

    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();


    @Override
    public void runOpMode() throws InterruptedException {
        RightMotor = hardwareMap.get(DcMotor.class, "RightMotor");
        LeftMotor = hardwareMap.get(DcMotor.class, "LeftMotor");
        RightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        LeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Very important initialization of the imu config
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);  // REMEMBER THIS TOO

        PID pid = new PID(3.4, 0.2, 0.2, 90);
        waitForStart();

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
        /*
        while (opModeIsActive()) {
            telemetry.addData("Path2",  "Running at %7d :%7d",
                    robot.Right.getCurrentPosition(),
                    robot.Left.getCurrentPosition());
            telemetry.update();
        }*/
    }
}
