package Atlas.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
import org.nd4j.linalg.dataset.DataSet;
import org.nd4j.linalg.factory.Nd4j;
import org.nd4j.linalg.learning.config.Adam;
import org.nd4j.linalg.lossfunctions.LossFunctions;
import org.opencv.core.Mat;

import java.util.Collection;
import java.lang.Math;

import Atlas.Autonomous.Init.Controller;
import Atlas.Autonomous.Init.HardwareAtlas;

@Autonomous(name = "ML", group = "bot")
public class AtlasControl extends LinearOpMode {
    private HardwareAtlas robot = new HardwareAtlas();
    double lower = -1, upper = 1;
    private double theta = 0, theta_dot = 0, prev_theta = 0;
    private double target = 100, norm_target = min_max(-180, 180, target, lower, upper);
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

        // Init DNN
        MultiLayerNetwork test = new MultiLayerNetwork(new NeuralNetConfiguration.Builder()
                .weightInit(WeightInit.XAVIER)
                .optimizationAlgo(OptimizationAlgorithm.STOCHASTIC_GRADIENT_DESCENT)
                .updater(new Adam(0.001))
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
                .layer(2, new OutputLayer.Builder().nIn(4).nOut(1)
                        .activation(Activation.IDENTITY)
                        .lossFunction(LossFunctions.LossFunction.MSE)
                        .build())
                .build());

        test.init();
        test.setListeners(new ScoreIterationListener(10));

        INDArray Y = Nd4j.create(new double[]{norm_target, 0}); // Target
        // DataSet data = new DataSet(); // Make the "dataset" to train on (this was for testing)
        double output;
        double norm_theta;
        double norm_theta_dot;
        int iteration = 0, orientation = 1;
        Collection<TrainingListener> score;
        while(opModeIsActive()) {
            // Get data and keep track of the iteration
            iteration++;
            theta = robot.imu.getAngularOrientation().firstAngle;
            theta_dot = prev_theta - theta;  // prev_theta and theta is already normalized so there is no need to again

            // So the DNN doesn't output anything greater or less than 1 and -1
            norm_theta = min_max(-180, 180, theta, lower, upper);

            // Check orientation area
            if(Math.abs(norm_theta) != norm_theta) {
                orientation = -1;
            } else {
                orientation = 1;
            }

            // Take absolute value of angle
            norm_theta = Math.abs(norm_theta);

            // Shift over target so there isn't 2 targets (shift 'graph' over so the target is at 0
            norm_theta += norm_target;

            // Make sure the value isn't outside the min or max bounds
            // Could also be an if statement, but the while just makes sure of it
            while(norm_theta > upper) {
                norm_theta -= Math.abs(upper);
            }
            while(norm_theta < lower) {
                norm_theta += Math.abs(lower);
            }

            // Input data
            // second line logs theta for the theta_dot calculation
            X = Nd4j.create(new double[]{norm_theta, theta_dot});
            prev_theta = theta;  // Update the previous_theta param (for theta dot calculation)


            // Train on data
            test.fit(X, Y);

            // Get the output and multiply it by the orientation
            output = test.output(X).getDouble(0);
            robot.Left.setPower(output * orientation);
            robot.Right.setPower(-output * orientation);

            // ?????
            score = test.getListeners();

            telemetry.addData("Iteration:", iteration);
            telemetry.addData("Target:", 100);
            telemetry.addData("Theta", theta);
            telemetry.addData("Theta dot", theta_dot);
            telemetry.addData("Output:", output);
            telemetry.addData("In case:", score.toString());
            telemetry.update();
        }
    }

    /**
     * Min max normalization
     * @param min the minimum value of the 'value' given
     * @param max the maximum value of the 'value' given
     * @param value the value to be normalized
     * @param lower the lower range to normalize around
     * @param upper the upper range to normalize around
     * @return the normalized value
     */
    private double min_max(double min, double max, double value, double lower, double upper) {
        return (upper - lower) * ((value - min) / (max - min)) + lower;
    }
}
