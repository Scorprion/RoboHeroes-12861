package Atlas.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.deeplearning4j.nn.api.OptimizationAlgorithm;
import org.deeplearning4j.nn.conf.MultiLayerConfiguration;
import org.deeplearning4j.nn.conf.NeuralNetConfiguration;
import org.deeplearning4j.nn.conf.layers.DenseLayer;
import org.deeplearning4j.nn.conf.layers.OutputLayer;
import org.deeplearning4j.nn.multilayer.MultiLayerNetwork;
import org.deeplearning4j.nn.weights.WeightInit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.nd4j.linalg.activations.Activation;
import org.nd4j.linalg.api.ndarray.INDArray;
import org.nd4j.linalg.factory.Nd4j;
import org.nd4j.linalg.learning.config.AdaGrad;

import Atlas.Autonomous.Init.Controller;
import Atlas.Autonomous.Init.HardwareAtlas;

@Autonomous(name = "ML", group = "bot")
public class ControlComputation extends LinearOpMode {
    private HardwareAtlas robot = new HardwareAtlas();
    private Controller c = new Controller();

    int iteration = 0;

    double angle = 0, angular_veloc = 0;
    double setpoint = 100; // The target/desired/reference angle
    double gamma = 0.95;  // The discount rate
    double epsilon = 1.0;  // Exploration rate
    double epsilon_decay = 0.95;  // Slowly decrease the epsilon parameter overtime
    double episilon_minimum = 0.01;  // Make sure epsilon doesn't decrease below this value
    INDArray X = Nd4j.create(new double[]{angle, angular_veloc});
    INDArray y = Nd4j.create(new double[]{setpoint});

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        angle = robot.imu.getAngularOrientation().firstAngle;
        MultiLayerConfiguration config = make_model(1e-02);
        MultiLayerNetwork model = new MultiLayerNetwork(config);
        model.init();
        while (opModeIsActive()) {
            iteration++;
            angle = robot.imu.getAngularOrientation().firstAngle;  // Keep track of the angle
            // The x rotation rate should be the yaw (which we want to control)
            angular_veloc = robot.imu.getAngularVelocity().toAngleUnit(AngleUnit.DEGREES).xRotationRate;

            // Update the X array
            X.putScalar(0, angle);
            X.putScalar(1, angular_veloc);


        }
    }

    private MultiLayerConfiguration make_model(double learning_rate) {
        MultiLayerConfiguration conf = new NeuralNetConfiguration.Builder()
                .weightInit(WeightInit.XAVIER)
                .updater(new AdaGrad(learning_rate))
                .optimizationAlgo(OptimizationAlgorithm.STOCHASTIC_GRADIENT_DESCENT)
                .dropOut(0.5)
                .list()
                .layer(0, new DenseLayer.Builder().nIn(2).nOut(4)
                        .activation(Activation.RELU)
                        .build())
                .layer(1, new DenseLayer.Builder().nIn(4).nOut(4)
                        .activation(Activation.RELU)
                        .build())
                .layer(2, new OutputLayer.Builder().nIn(4).nOut(1)
                        .activation(Activation.IDENTITY)
                        .build())
                .build();
        return conf;
    }
}
