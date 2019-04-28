package Atlas.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.deeplearning4j.nn.api.OptimizationAlgorithm;
import org.deeplearning4j.nn.conf.NeuralNetConfiguration;
import org.deeplearning4j.nn.conf.layers.DenseLayer;
import org.deeplearning4j.nn.conf.layers.OutputLayer;
import org.deeplearning4j.nn.multilayer.MultiLayerNetwork;
import org.deeplearning4j.nn.weights.WeightInit;
import org.deeplearning4j.optimize.api.TrainingListener;
import org.deeplearning4j.optimize.listeners.ScoreIterationListener;
import org.nd4j.linalg.activations.Activation;
import org.nd4j.linalg.api.ndarray.INDArray;
import org.nd4j.linalg.factory.Nd4j;
import org.nd4j.linalg.learning.config.Adam;
import org.nd4j.linalg.lossfunctions.LossFunctions;

import java.util.Collection;

import Atlas.Autonomous.Init.Controller;
import Atlas.Autonomous.Init.HardwareAtlas;

public class AtlasControl extends LinearOpMode {
    private HardwareAtlas robot = new HardwareAtlas();
    private double theta = 0, theta_dot = 0, prev_theta = 0;
    private Controller c = new Controller();

    // Should be a 1x1 "array" of the target angle
    INDArray X = Nd4j.create(2,1);
    INDArray A = Nd4j.zeros(2,1);
    INDArray B = Nd4j.zeros(2,1);
    INDArray Q = Nd4j.zeros(2,1);
    INDArray R = Nd4j.zeros(2,1);

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
        INDArray y = Nd4j.create(new double[]{100, 0}); // Target

        // Init DNN
        MultiLayerNetwork test = new MultiLayerNetwork(new NeuralNetConfiguration.Builder()
                .weightInit(WeightInit.XAVIER)
                .optimizationAlgo(OptimizationAlgorithm.STOCHASTIC_GRADIENT_DESCENT)
                .updater(new Adam(0.01))
                .dropOut(0.5)
                .list()

                // Input layer (angle and derivative of angle (angular velocity)
                .layer(0, new DenseLayer.Builder().nIn(2).nOut(4)
                        .activation(Activation.RELU)
                        .build())

                // Make a hidden layer
                .layer(1, new DenseLayer.Builder().nIn(4).nOut(4)
                        .activation(Activation.RELU)
                        .build())

                // Output layer
                .layer(2, new OutputLayer.Builder().nIn(4).nOut(2)
                        .activation(Activation.SIGMOID)
                        .lossFunction(LossFunctions.LossFunction.MSE)
                        .build())
                .build());

        test.init();
        test.setListeners(new ScoreIterationListener(10));

        double output, loss;
        Collection<TrainingListener> score;
        while(opModeIsActive()) {
            theta = robot.imu.getAngularOrientation().firstAngle;
            theta = c.eulerNormalize(theta);
            theta_dot = prev_theta - theta;
            X = Nd4j.create(new double[]{theta, theta_dot});
            prev_theta = theta;

            test.fit(X, y);
            output = test.output(X).getDouble(0);
            robot.Left.setPower(output);
            robot.Right.setPower(output);

            score = test.getListeners();
            loss = (double) score.toArray()[0];

            telemetry.addData("Target:", 100);
            telemetry.addData("Theta", theta);
            telemetry.addData("Theta dot", theta_dot);
            telemetry.addData("Output:", output);
            telemetry.addData("Score:", loss);
            telemetry.addData("In case:", score);
            telemetry.update();
        }
    }
}
