package Atlas.Autonomous.Init;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import Atlas.Autonomous.Init.HardwareAtlas;

public class AggregatedClass extends LinearOpMode {
    /**
     * A program that houses all of our methods needed to run our robot's
     * autonomous programs
     * <p>
     * Since we couldn't use interfaces or anything like that to be able to implement different set
     * methods like
     *
     * @param encoderDrive
     */

    //Using our robot's hardware
    HardwareAtlas robot = new HardwareAtlas();

    //Defining final variables for the encoders
    final double countsPerRot = 2240; // The counts per rotation
    final double gearBoxRatio = 0.025; // The gear box ratio for the motors
    final double wheelDiamInch = 4; // The diameter of the Atlas wheels for finding the circumference
    final double countsPerInch = (countsPerRot * gearBoxRatio) / (wheelDiamInch * 3.1415);

    PIDController pidRotate, pidDrive;
    Orientation lastAngles = new Orientation();
    double globalAngle, power = .30, correction;
    boolean aButton, bButton, touched;
    
    @Override
    public void runOpMode() throws InterruptedException {
        //Initializing
        robot.init(hardwareMap);
        
        
        
        pidRotate = new PIDController(0.005, 0, 0);
        pidDrive = new PIDController(0.05, 0,0);
    }

    /**
     * Encoder method for controlling our autonomous programs
     *
     * @param speed    The speed at which the motors turn at
     * @param linch    the distance for the left motor to turn (in inches)
     * @param rinch    the distance for the right motor to turn (in inches)
     * @param timeoutS the amount of time the encoder gives to the motor to move/turn to the target
     *                 speed (in seconds)
     */
    public void encoderDrive(double speed, double linch, double rinch,
                             double timeoutS) {

        int newLeftTarget, newRightTarget;

        if (opModeIsActive()) {
            //Get new targets for the wheels based on the wheel's countsPerInch
            //and how far you still need to go off the motors' current position
            newLeftTarget = robot.Left.getCurrentPosition() + (int) (linch * countsPerInch);
            newRightTarget = robot.Right.getCurrentPosition() + (int) (rinch * countsPerInch);
            robot.Left.setTargetPosition(newLeftTarget);
            robot.Right.setTargetPosition(newRightTarget);

            //Set the mode of the encoders to "RUN_TO_POSITION" mode
            robot.Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            //Resetting timer
            robot.runtime.reset();

            //Set the speed of the motors to the speed specified
            robot.Left.setPower(Math.abs(speed));
            robot.Right.setPower(Math.abs(speed));

            //While the program is still running, the runtime seconds is less than
            //the given time in seconds, and both the left and right motors are busy, then
            //continuously display the first 4 digits (%4d) of the new target
            //and the current position of the motors to the phone/drivers
            while (opModeIsActive() && (robot.runtime.seconds() < timeoutS) &&
                    (robot.Left.isBusy() && robot.Right.isBusy())) {
                telemetry.addData("Path 1", "Running to %4d :%4d",
                        newLeftTarget, newRightTarget);
                telemetry.addData("Path 2", "Running to %4d :%4d",
                        robot.Left.getCurrentPosition(), robot.Right.getCurrentPosition());
                telemetry.update();
            }

            //Stop all motion when done
            robot.Left.setPower(0);
            robot.Right.setPower(0);

            robot.Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }


    public class PIDController {
        //Defining all of our needed values for PID
        public double m_P; //Factor for "proportional" control
        public double m_I; //Factor for "integral" control
        public double m_D; //Factor for "derivative" control
        public double m_input; //Sensor input for pid controller
        public double m_maxOut = 1; //|maximum output| (absolute value of the maximum output)
        public double m_minOut = -1; //|minimum output| (absolute value of the minimum output)
        public double m_maxIn = 0; //Maximum input - limit setpoint to this
        public double m_minIn = 0; //Minimum input - limit setpoint to this
        public boolean m_continuous = false; //Do the endpoints wrap around? (eg. Absolute encoder)
        public boolean m_enabled = false; //Is the PID controller enabled?
        public double m_prevError = 0; //The prior sensor input (used to compute velocity)
        public double m_totalError = 0; //the sum of the errors for use in the integral calc
        public double m_tolerance = 0.05; //the percentage error that is considered on target
        public double m_setpoint = 0;
        public double m_error = 0;
        public double m_result = 0;

    /*
    ------------------------------
    |                            |
    |       Our PID methods      |
    |                            |
    ------------------------------
     */

        public void rotate(int degrees, double power) {
            // restart imu angle tracking.
            resetAngle();

            // start pid controller. PID controller will monitor the turn angle with respect to the
            // target angle and reduce power as we approach the target angle with a minimum of 20%.
            // This is to prevent the robots momentum from overshooting the turn after we turn off the
            // power. The PID controller reports atTarget() = true when the difference between turn
            // angle and target angle is within 2% of target (tolerance). This helps prevent overshoot.
            // The minimum power is determined by testing and must enough to prevent motor stall and
            // complete the turn. Note: if the gap between the starting power and the stall (minimum)
            // power is small, overshoot may still occur. Overshoot is dependant on the motor and
            // gearing configuration, starting power, weight of the robot and the on target tolerance.

            pidRotate.reset();
            pidRotate.setSetpoint(degrees);
            pidRotate.setInRange(0, 90);
            pidRotate.setOutRange(.20, power);
            pidRotate.setTolerance(2);
            pidRotate.enable();

            // getAngle() returns + when rotating counter clockwise (left) and - when rotating
            // clockwise (right).

            // rotate until turn is completed.

            if (degrees < 0) {
                // On right turn we have to get off zero first.
                while (opModeIsActive() && getAngle() == 0) {
                    robot.Left.setPower(-power);
                    robot.Right.setPower(power);
                    sleep(100);
                }

                do {
                    power = pidRotate.performPID(getAngle()); // power will be - on right turn.
                    robot.Left.setPower(power);
                    robot.Right.setPower(-power);
                } while (opModeIsActive() && !pidRotate.atTarget());
            } else    // left turn.
                do {
                    power = pidRotate.performPID(getAngle()); // power will be + on left turn.
                    robot.Left.setPower(power);
                    robot.Right.setPower(-power);
                } while (opModeIsActive() && !pidRotate.atTarget());

            // turn the motors off.
            robot.Right.setPower(0);
            robot.Left.setPower(0);

            // wait for rotation to stop.
            sleep(500);

            // reset angle tracking on new heading.
            resetAngle();
        }

        public void resetAngle()
        {
            lastAngles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            globalAngle = 0;
        }

        public double performPID(double input)
        {
            setInput(input);
            return performPID();
        }
        
        public double performPID()
        {
            calculatePID();
            return m_result;
        }
        
        double getAngle() {
            // We experimentally determined the Z axis is the axis we want to use for heading angle.
            // We have to process the angle because the imu works in euler angles so the Z axis is
            // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
            // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

            Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

            if (deltaAngle < -180)
                deltaAngle += 360;
            else if (deltaAngle > 180)
                deltaAngle -= 360;

            globalAngle += deltaAngle;

            lastAngles = angles;

            return globalAngle;
        }

        /**
         * Assigning the P, I, and D coefficients to those passed to this method
         *
         * @param Kp the proportional coefficient
         * @param Ki the integral coefficient
         * @param Kd the derivative coefficient
         */
        public PIDController(double Kp, double Ki, double Kd) {
            m_P = Kp;
            m_I = Ki;
            m_D = Kd;
        }

        /**
         * Calculating for PID control on our IMU sensor
         */
        public void calculatePID() {
            int sign = 1;

            // Calculate the error signal
            m_error = m_setpoint - m_input;

            // If continuous is set to true allow wrap around
            if (m_continuous) {
                if (Math.abs(m_error) > (m_maxIn - m_minIn) / 2) {
                    if (m_error > 0)
                        m_error = m_error - m_maxIn + m_minIn;
                    else
                        m_error = m_error + m_maxIn - m_minIn;
                }
            }

            // Integrate the errors as long as the upcoming integrator does
            // not exceed the minimum and maximum output thresholds.

            if ((Math.abs(m_totalError + m_error) * m_I < m_maxOut) &&
                    (Math.abs(m_totalError + m_error) * m_I > m_minOut))
                m_totalError += m_error;

            // Perform the primary PID calculation
            m_result = m_P * m_error + m_I * m_totalError + m_D * (m_error - m_prevError);

            // Set the current error to the previous error for the next cycle.
            m_prevError = m_error;

            if (m_result < 0) sign = -1;    // Record sign of result.

            // Make sure the final result is within bounds. If we constrain the result, we make
            // sure the sign of the constrained result matches the original result sign.
            if (Math.abs(m_result) > m_maxOut)
                m_result = m_maxOut * sign;
            else if (Math.abs(m_result) < m_minOut)
                m_result = m_minOut * sign;
        }

        /**
         * Setting up the minimum and maximum input/output ranges
         *
         * @param minIn  the minimum expected value from the input (always positive)
         * @param maxIn  the maximum expected value from the input (always positive)
         */
        public void setInRange(double minIn, double maxIn) {
            m_minIn = Math.abs(minIn);
            m_maxIn = Math.abs(maxIn);
            setSetpoint(m_setpoint);
        }

        public void setOutRange(double minOut, double maxOut) {
            m_minOut = minOut;
            m_maxOut = maxOut;
        }

        /**
         * Set the input value to be used by the next call of calculatePID()
         *
         * @param input the input value for the PID calculations
         */
        public void setInput(double input) {
            //Just to set the output value negative or not when we need to
            int sign = 1;

            if (m_enabled) {

                //Find the error which is equal to the goal minus our input value (our position)
                m_error = m_setpoint - m_input;

                if (m_continuous) {
                    if (Math.abs(m_error) > (m_maxIn - m_minIn) / 2) {
                        if (m_error > 0) {
                            m_error = m_error - m_maxIn + m_minIn;
                        }
                    }
                }
            }
        }

        /**
         * Setting the setpoint (target value) the robot needs to aim for
         *
         * @param setpoint the target
         */
        public void setSetpoint(double setpoint) {
            int sign = 1;

            if (m_maxIn > m_minIn) {
                if (setpoint < 0) {
                    sign = -1;
                }

                if (Math.abs(setpoint) > m_maxIn) {
                    m_setpoint = m_maxIn * sign;
                } else if (Math.abs(setpoint) < m_minIn) {
                    m_setpoint = m_minIn * sign;
                } else {
                    m_setpoint = setpoint;
                }
            } else {
                m_setpoint = setpoint;
            }
        }

        /**
         * Set the amount of tolerance for the output to be considered at the target value
         *
         * @param percentage overriding the default tolerance of 0.05 (5%)
         */
        public void setTolerance(double percentage) {
            m_tolerance = percentage;
        }

        /**
         * Detect if the the desired target was "hit" (we're at the target)
         *
         * @return true or false, depending on if the error was less than the tolerance multiplied by
         * the difference between the max and min input
         */
        public boolean atTarget() {
            return (Math.abs(m_error) < Math.abs(m_tolerance / 100 * (m_maxIn - m_minIn)));
        }
        
        /**
         * Enable the PID to start running
         */
        public void enable() {
            m_enabled = true;
        }

        /**
         * Disable the PID to stop running
         */
        public void disable() {
            m_enabled = false;
        }

        /**
         * Resetting the variables for another measurement
         */
        public void reset() {
            m_prevError = 0;
            m_totalError = 0;
            m_result = 0;
        }
    }
}
