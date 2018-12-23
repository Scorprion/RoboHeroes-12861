package TestBot.Init;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class HardwareTestBot {
    //Sensors
    public NormalizedColorSensor ColorSensor;
    //public DistanceSensor DistanceSensor;

    public DcMotor Right;
    public DcMotor Left;

    //IMU sensor
    public BNO055IMU imu;

    // State used for updating telemetry
    public Orientation angles;
    public Acceleration gravity;

    // The elapsed time
    public ElapsedTime runtime = new ElapsedTime();
    public ModernRoboticsI2cGyro gyro;

    HardwareMap hwMap = null;

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;
        ColorSensor = hwMap.get(NormalizedColorSensor.class, "ColorSensor");
        //DistanceSensor = hwMap.get(DistanceSensor.class, "DistanceSensor");

        //IMU sensor
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        Right = hwMap.get(DcMotor.class, "Right");
        Right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Left = hwMap.get(DcMotor.class, "Left");
        Left.setDirection(DcMotorSimple.Direction.REVERSE);
        Left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Right.setPower(0);
        Left.setPower(0);
    }
}
