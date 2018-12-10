package Atlas.Autonomous.MoveMethods;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import Atlas.HardwareAtlas;

public class AtlasIMUTurn extends LinearOpMode {
    HardwareAtlas robot = new HardwareAtlas();
    public void gyroTurn() {

    }


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
    }
}
