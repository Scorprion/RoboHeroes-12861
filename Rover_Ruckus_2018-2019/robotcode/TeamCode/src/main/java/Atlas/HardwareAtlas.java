package Atlas;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.HardwareMapInit;

public class HardwareAtlas {
    //Sensors
    public NormalizedColorSensor ColorSensor;
    //public DistanceSensor DistanceSensor;

    //Servos
    private Servo Clamp;
    public Servo Marker;

    //Right motors
    private DcMotor RShoulder;
    private DcMotor RElbow;
    public DcMotor Right;

    //Left motors
    private DcMotor LShoulder;
    private DcMotor LElbow;
    public DcMotor Left;

    //IMU sensor
    public BNO055IMU imu;

    // State used for updating telemetry
    public Orientation angles;
    public Acceleration gravity;


    HardwareMap hwMap = null;

    public HardwareAtlas(HardwareMap ahwMap) {
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

        Clamp = hwMap.get(Servo.class, "Clamp");
        Marker = hwMap.get(Servo.class, "Marker");

        RShoulder = hwMap.get(DcMotor.class, "RShoulder");
        RElbow = hwMap.get(DcMotor.class, "RElbow");
        Right = hwMap.get(DcMotor.class, "Right");
        Right.setDirection(DcMotorSimple.Direction.REVERSE);

        LShoulder = hwMap.get(DcMotor.class, "LShoulder");
        LElbow = hwMap.get(DcMotor.class, "LElbow");
        Left = hwMap.get(DcMotor.class, "Left");

        RShoulder.setPower(0);
        RElbow.setPower(0);
        Right.setPower(0);
        LShoulder.setPower(0);
        LElbow.setPower(0);
        Left.setPower(0);

        // Make the motors not use encoders by default but you can set it to use encoders in the
        // program manually with "RUN_USING_ENCODER" and "STOP_AND_RESET_ENCODER"
        RShoulder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RElbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LShoulder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LElbow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
