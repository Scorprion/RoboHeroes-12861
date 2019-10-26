package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Autonomous.Init.Aggregated;
import org.firstinspires.ftc.teamcode.Autonomous.Init.HardwareClass;
import org.firstinspires.ftc.teamcode.Autonomous.Init.PID;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

@Autonomous(name = "Auto_A", group = "Autonomous")
public class Auto_A extends Aggregated {

    private double speed = 0.1, pidOutput = 0;
    private PID pid = new PID(0.5, 0.5, 0, 0);
    private VectorF locationV;
    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime Progressor = new ElapsedTime();
    private boolean NoMovingMotor = true;
    private boolean SecondPhase = false;

    @Override
    public void runOpMode() throws InterruptedException {
        //Initialization
        robot.init(hardwareMap);
        waitForStart();


        //Forward to prepare for Vuforia sensing
        encoderDrives(0.3, 28, 28, pid);


        //90 Degree turn
        pid.setParams(1.34, 0.5, 0,90);
        StopMotors();
        timer.reset();

        //Forward to sense Stones
        while(vuforia() == null) {
            robot.Left.setPower(0.3);
            robot.Right.setPower(0.3);
        }NoMovingMotor = false;

        timer.reset();


        //Angle Display
        while(timer.milliseconds() < 2500 && opModeIsActive()) {
            telemetry.addData("Angle: ", robot.imu.getAngularOrientation().firstAngle);
            telemetry.update();
            pidOutput = pid.getPID(robot.imu.getAngularOrientation().firstAngle);
            robot.Left.setPower(-pidOutput);
            robot.Right.setPower(pidOutput);
        }

        while(opModeIsActive() && NoMovingMotor){
            StopArm();
        }

        while(opModeIsActive()) {
            telemetry.addData("Seconds:",  Progressor.seconds());
        }
        /*
        while (opModeIsActive()) {
            locationV = vuforia();
            if (locationV != null) {
                pid.getPID(locationV.get(2));
                robot.Left.setPower(speed + pid.PIDout);
                robot.Right.setPower(speed + pid.PIDout);
            } else {
                robot.Left.setPower(0);
                robot.Right.setPower(0);
            }

        }*/



    }
}

