package Atlas;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.HardwareMapInit;

public class HardwareAtlas {
    //Sensors
    public NormalizedColorSensor ColorSensor;
    public DistanceSensor DistanceSensor;

    //Servos
    public Servo Clamp;
    public Servo Marker;

    //Right motors
    public DcMotor RShoulder;
    public DcMotor RElbow;
    public DcMotor Right;

    //Left motors
    public DcMotor LShoulder;
    public DcMotor LElbow;
    public DcMotor Left;

    HardwareMap hwMap = null;

    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;
        ColorSensor = hwMap.get(NormalizedColorSensor.class, "ColorSensor");
        DistanceSensor = hwMap.get(DistanceSensor.class, "DistanceSensor");

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
