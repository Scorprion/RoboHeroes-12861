package Atlas.Autonomous.Init;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import Atlas.Autonomous.Init.HardwareAtlas;

public class AggregatedClass extends LinearOpMode {
    /**
     * A program that houses all of our methods needed to run our robot's
     * autonomous programs
     *
     * Since we couldn't use interfaces or anything like that to be able to implement different set
     * methods like
     * @param encoderDrive
     *
     *
     *
     *
    */

    //Using our robot's hardware
    HardwareAtlas robot = new HardwareAtlas();

    //Defining final variables for the encoders
    final double countsPerRot = 2240; // The counts per rotation
    final double gearBoxRatio = 0.025; // The gear box ratio for the motors
    final double wheelDiamInch = 4; // The diameter of the Atlas wheels for finding the circumference
    final double countsPerInch = (countsPerRot * gearBoxRatio) / (wheelDiamInch * 3.1415);

    @Override
    public void runOpMode() throws InterruptedException {
        //Initializing
        robot.init(hardwareMap);
    }

    /** Encoder method for controlling our autonomous programs
     * @param speed The speed at which the motors turn at
     * @param linch the distance for the left motor to turn (in inches)
     * @param rinch the distance for the right motor to turn (in inches)
     * @param timeoutS the amount of time the encoder gives to the motor to move/turn to the target
     *                 speed (in seconds)
     */
    public void encoderDrive(double speed, double linch, double rinch,
                             double timeoutS) {

        int newLeftTarget, newRightTarget;

        if (opModeIsActive()) {
            //Get new targets for the wheels based on the wheel's countsPerInch
            //and how far you still need to go off the motors' current position
            newLeftTarget = robot.Left.getCurrentPosition() + (int)(linch * countsPerInch);
            newRightTarget = robot.Right.getCurrentPosition() + (int)(rinch * countsPerInch);
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

    /** Assigning the P, I, and D coefficients to those passed to this method
     * @param Kp the proportional coefficient
     * @param Ki the integral coefficient
     * @param Kd the derivative coefficient
     */
    public void PIDInit(double Kp, double Ki, double Kd) {
        m_P = Kp;
        m_I = Ki;
        m_D = Kd;
    }

    /** Calculating for PID control on our IMU sensor
     *
     */
    public void calculatePID() {
        
    }

    /** Setting up the minimum and maximum input/output ranges
     * @param minIn the minimum expected value from the input (always positive)
     * @param maxIn the maximum expected value from the input (always positive)
     * @param minOut the minimum value to write to the output (always positive)
     * @param maxOut the maximum value to write to the output (always positive)
     */
    public void setInOutRange(double minIn, double maxIn, double minOut, double maxOut) {
        
    }

    /** Set the input value to be used by the next call of calculatePID()
     * @param input the input value for the PID calculations
     */
    public void setInput(double input) {
        //Just to set the output value negative or not when we need to
        int sign = 1;

        if(m_enabled) {

            //Find the error which is equal to the goal minus our input value (our position)
            m_error = m_setpoint - m_input;

            if(m_continuous) {
                if(Math.abs(m_error) > (m_maxIn - m_minIn) / 2) {
                    if(m_error > 0) {
                        m_error = m_error - m_maxIn + m_minIn;
                    }
                }
            }
        }
    }

    /** Setting the setpoint (target value) the robot needs to aim for
     *
     */
    public void setSetpoint() {
        
    }

    /** Set the amount of tolerance for the output to be considered at the target value
     * @param percentage overriding the default tolerance of 0.05 (5%)
     */
    public void setTolerance(double percentage) {
        m_tolerance = percentage;
    }

    /** Detect if the the desired target was "hit" (we're at the target)
     * @return true or false, depending on if the error was less than the tolerance multiplied by
     *         the difference between the max and min input
     */
    public boolean atTarget() {
        return(Math.abs(m_error) < Math.abs(m_tolerance / 100 * (m_maxIn - m_minIn)));
    }

    /** Enable the PID to start running
     */
    public void enable() {
        m_enabled = true;
    }

    /** Disable the PID to stop running
     */
    public void disable() {
        m_enabled = false;
    }

    /** Resetting the variables for another measurement
     */
    public void reset() {
        m_prevError = 0;
        m_totalError = 0;
        m_result = 0;
    }
 }
