package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Autonomous.Init.Aggregated;
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

    private double speed = 0.1;
    private PID pid = new PID(0.5, 0.5, 0, 0);
    private VectorF locationV;

    @Override
    public void runOpMode() throws InterruptedException {
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
        }
    }
}

