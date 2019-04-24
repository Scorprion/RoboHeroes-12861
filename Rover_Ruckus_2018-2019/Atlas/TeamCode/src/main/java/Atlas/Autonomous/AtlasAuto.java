package Atlas.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.files.DataLogger;

import java.io.IOException;

import Atlas.Autonomous.Init.Aggregated;

@Autonomous(name = "AtlasAuto", group = "Atlas")
public class AtlasAuto extends Aggregated {

    final double countsPerRot = 2240; // The counts per rotation
    final double gearBoxRatio = 0.5; // The gear box ratio for the motors
    final double wheelDiamInch = 4; // The diameter of the Atlas wheels for finding the circumference
    public final double countsPerInch = (countsPerRot * gearBoxRatio) / (wheelDiamInch * 3.1415);
    private double output = 1; // Needs to be 100 to get passed the "while" statement's conditions
    private double angle = 0;

    private ElapsedTime passed = new ElapsedTime();
    DataLogger d;
    {
        try {
            d = new DataLogger("Identification.csv");
            d.addHeaderLine("Iteration, Time, Angle, Power");
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();
        passed.reset();

        int iteration = 0;
        double speed = 0, past = 0, dt = 1, past_dist = 0;
        try {
            DataLogger d = new DataLogger("Test.csv");
            d.addHeaderLine("Iteration, Time, Speed, Power");
        } catch (IOException e) {
            e.printStackTrace();
        }


        while(opModeIsActive()) {
            iteration++;
            past = passed.milliseconds();
            past_dist = robot.Right.getCurrentPosition() / countsPerInch;

            if(round(passed.seconds(), 0) % 3 == 0) {
                robot.Left.setPower(round(passed.seconds() / 10, 1));
                robot.Right.setPower(-round(passed.seconds() / 10, 1));
            }
            speed = ((robot.Right.getCurrentPosition() / countsPerInch) - past_dist) / dt;
            telemetry.addData("Seconds:", round(passed.seconds(), 0));
            telemetry.addData("Iteration:", iteration);
            telemetry.addData("Speed:", speed);
            telemetry.addData("Power:", round(passed.seconds() / 10, 0));
            telemetry.update();
            try {
                d.addDataLine(iteration + "," + passed.milliseconds() + "," + speed + "," + passed.seconds() / 10);
            } catch (IOException e) {
                e.printStackTrace();
            }
            dt = passed.milliseconds() - past;
        }
        d.close();

    }
}
