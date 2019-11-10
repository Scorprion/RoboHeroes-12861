
@Autonomous(name = "HermesA", group = "Autonomous")
public class HermesA extends HermesAggregated {
    private double speed = 0.1, pidOutput = 0;
    private VectorF locationV;
    public boolean VuforiaFound = false;
    @Override
        robot.init(hardwareMap);
    public void runOpMode() throws InterruptedException {

        waitForStart();
        mecanumMove(0.5, 30, 10);
    }
}