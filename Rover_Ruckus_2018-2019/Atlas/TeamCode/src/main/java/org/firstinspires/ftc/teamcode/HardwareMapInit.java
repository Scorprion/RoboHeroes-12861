package org.firstinspires.ftc.teamcode;

import android.view.View;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

public class HardwareMapInit {

    public NormalizedColorSensor Color_Sensor;
    public NormalizedColorSensor ColorGround;
    public NormalizedColorSensor Color1;
    public NormalizedColorSensor Color2;
    public DcMotor Right;
    public DcMotor Left;
    public View relativeLayout;
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();
    public ModernRoboticsI2cGyro gyro;
    BNO055IMU imu;

    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        Color_Sensor = hwMap.get(NormalizedColorSensor.class, "ColorSensor");
        ColorGround = hwMap.get(NormalizedColorSensor.class, "ColorGround");
        Color1 = hwMap.get(NormalizedColorSensor.class, "Color1");
        Color2 = hwMap.get(NormalizedColorSensor.class, "Color2");

        Right = hwMap.get(DcMotor.class, "Right");
        Left = hwMap.get(DcMotor.class, "Left");

        Right.setDirection(DcMotorSimple.Direction.REVERSE);


        // Set them to 0
        Right.setPower(0);
        Left.setPower(0);

        // Make them not use encoders by default but you can set it to use encoders in the program
        // manually with "RUN_USING_ENCODER" and "STOP_AND_RESET_ENCODER"
        Right.setMode(DcMotor.RunMode. RUN_WITHOUT_ENCODER);
        Left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}




