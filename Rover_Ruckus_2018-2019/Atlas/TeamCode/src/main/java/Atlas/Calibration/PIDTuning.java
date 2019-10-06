package Atlas.Calibration;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import Atlas.Autonomous.Init.Aggregated;
import Atlas.Autonomous.Init.PID;

@Autonomous(name = "PID Tuner", group = "Calibration")
public class PIDTuning extends Aggregated {
    final double countsPerRot = 2240; // The counts per rotation
    final double gearReductionRatio = 0.5; // The gear box ratio for the motors
    final double wheelDiamInch = 4; // The diameter of the Atlas wheels for finding the circumference
    public final double countsPerInch = 47.5; //(countsPerRot * gearReductionRatio) / (wheelDiamInch * 3.1415);
    private double angle = 0;
    private double pidOutput = 0;
    private double speed = 0.1;

    private DcMotor RightMotor;
    private DcMotor LeftMotor;
    private BNO055IMU imu;

    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

    ElapsedTime timer = new ElapsedTime();

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

        waitForStart();
        //encoderDrives(0.2, 60, 60, pid);
        while(opModeIsActive()) {
            if(timer.seconds() < 3) {
                timer.reset();
                speed += 0.05;
            }

            LeftMotor.setPower(speed);
            RightMotor.setPower(-speed);

            telemetry.log().add("Time", timer.seconds(), "Input", speed, "Output", imu.getAngularOrientation().firstAngle);
            telemetry.addData("Current Angle: ", imu.getAngularOrientation().firstAngle);
            telemetry.update();
        }
    }
}
