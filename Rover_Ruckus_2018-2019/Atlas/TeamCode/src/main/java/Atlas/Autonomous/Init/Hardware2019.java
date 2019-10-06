package Atlas.Autonomous.Init;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Hardware2019 {

    public DcMotor Right;
    public DcMotor Left;

    public CRServo ClawR;
    public CRServo ClawL;

    public BNO055IMU imu;

    public Orientation angles;
    public Acceleration gravity;

    // The elapsed time
    public ElapsedTime runtime = new ElapsedTime();

    public void init(HardwareMap ahwMap) {

        //DistanceSensor = ahwMap.get(DistanceSensor.class, "DistanceSensor");

        //IMU sensor
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        imu = ahwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        Right = ahwMap.get(DcMotor.class, "Right");
        Right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Left = ahwMap.get(DcMotor.class, "Left");
        Left.setDirection(DcMotorSimple.Direction.REVERSE);
        Left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Right.setPower(0);
        Left.setPower(0);

    }
}
