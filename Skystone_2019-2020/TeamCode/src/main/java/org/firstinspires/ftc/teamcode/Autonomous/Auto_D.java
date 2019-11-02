package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

@Autonomous(name = "Auto_D", group = "Autonomous")
public class Auto_D extends Aggregated {
    private double speed = 0.1, pidOutput = 0;
    private PID pid = new PID(0.5, 0.5, 0, 0);
    private double locationV = -1000;
    public boolean VuforiaFound = false;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        waitForStart();

        robot.Arm.setPower(0.5);

        encoderDrives(0.3, 22, 22, 2);
        /*pid.setParams(1.45, 0.5, 0, 90);
        run_pid(0, 5500, pid, true);

        vuforia();
        locationV = translation.get(2);
        if (locationV != 0 && locationV != -1000) {
            run_pid(0, 1500, false, 0.5, 0.3, 0, 0);
        }

        pid.setParams(1.52, 0.5, 0, 0);
        run_pid(0, 5500, pid, true);*/
        encoderDrives(0.3, 10, 10,2);
        robot.Arm.setPower(-1);
        sleep(250);
        robot.Arm.setPower(0);
        encoderDrives(0.3, -10, -10,1);
        encoderDrives(0.2, 10, -10,1);
        encoderDrives(0.5, 103,103,4);
        robot.Arm.setPower(0.7);
        sleep(750);
        robot.Arm.setPower(0);
        encoderDrives(0.5, -60, -60,3.5);
    }
}