package org.firstinspires.ftc.teamcode.PikaMimic.Autonomous.Init;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Pikaware {
    public DcMotor Left;
    public DcMotor Right;

    public void init(HardwareMap ahwMap){
        Right = ahwMap.get(com.qualcomm.robotcore.hardware.DcMotor.class, "Right");
        Right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Right.setDirection(DcMotorSimple.Direction.REVERSE);

        Left =  ahwMap.get(DcMotor.class, "Left");
        Right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
