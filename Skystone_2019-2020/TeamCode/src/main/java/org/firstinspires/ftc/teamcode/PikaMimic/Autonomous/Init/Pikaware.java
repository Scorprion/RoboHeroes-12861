package org.firstinspires.ftc.teamcode.PikaMimic.Autonomous.Init;

public class Pikaware {
    public Left DcMotor;
    public Right DcMotor;

    public void init(HardwareMap ahwMap){
        Right = ahwMap.get(DcMotor.class, "Right");
        Right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Right.setDirection(DcMotorSimple.Direction.REVERSE);

        Left =  ahwMap.get(DcMotor.class, "Left");
        Right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
