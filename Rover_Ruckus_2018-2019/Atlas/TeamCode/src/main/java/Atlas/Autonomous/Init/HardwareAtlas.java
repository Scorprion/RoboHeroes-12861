package Atlas.Autonomous.Init;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.HardwareMapInit;

import Atlas.Autonomous.SensorTesting.PIDIMU;

public class HardwareAtlas {
    //Sensors
    public NormalizedColorSensor ColorSensor;
    public NormalizedColorSensor BottomCS;
    //public DistanceSensor DistanceSensor;

    //Servos
    public Servo LClamp;
    public Servo Marker;
    public Servo Sliding;
    public CRServo Latching;

    //Right motors
    public DcMotor Right;

    //Left motors
    public DcMotor LShoulder;
    public DcMotor LElbow;
    public DcMotor Left;
    public DcMotor Middle;

    //Winch
    public DcMotor Winch;

    //IMU sensor
    public BNO055IMU imu;

    // State used for updating telemetry
    public Orientation angles;
    public Acceleration gravity;

    // The elapsed time
    public ElapsedTime runtime = new ElapsedTime();

    HardwareMap hwMap = null;

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;
        ColorSensor = hwMap.get(NormalizedColorSensor.class, "ColorSensor");
        BottomCS = hwMap.get(NormalizedColorSensor.class, "BottomCS");
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

        LClamp = hwMap.get(Servo.class, "LClamp");
        Latching = hwMap.get(CRServo.class, "Latching");
        Marker = hwMap.get(Servo.class, "Marker");
        Sliding = hwMap.get(Servo.class, "Sliding");

        Right = hwMap.get(DcMotor.class, "Right");
        Right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        LShoulder = hwMap.get(DcMotor.class, "LShoulder");
        LElbow = hwMap.get(DcMotor.class, "LElbow");
        Left = hwMap.get(DcMotor.class, "Left");
        Left.setDirection(DcMotorSimple.Direction.REVERSE);
        Left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Winch = hwMap.get(DcMotor.class,"Winch");


        Right.setPower(0);
        LShoulder.setPower(0);
        LElbow.setPower(0);
        LElbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Left.setPower(0);

        // Make the motors not use encoders by default but you can set it to use encoders in the
        // program manually with "RUN_USING_ENCODER" and "STOP_AND_RESET_ENCODER"
        //Right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LShoulder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LElbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
