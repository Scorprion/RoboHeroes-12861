package STC_2019;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


public class HardwareSTC {

    public DcMotor Left;
    public DcMotor Right;

    public CRServo Lift;
    public CRServo Clamp;

    public ElapsedTime runtime = new ElapsedTime();

    public NormalizedColorSensor Color1;
    public NormalizedColorSensor Color2;

    public void init(HardwareMap STCMap) {
        Left = STCMap.get(DcMotor.class, "Left");
        Left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Right = STCMap.get(DcMotor.class, "Right");
        Right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Right.setDirection(DcMotorSimple.Direction.REVERSE);

        Lift = STCMap.get(CRServo.class, "Lift");
        Clamp = STCMap.get(CRServo.class, "Clamp");

        Color1 = STCMap.get(NormalizedColorSensor.class, "Color1");
        Color2 = STCMap.get(NormalizedColorSensor.class, "Color1");

        Right.setPower(0);
        Left.setPower(0);
    }
}
