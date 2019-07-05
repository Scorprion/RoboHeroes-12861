package Atlas.JavaClass;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import Atlas.Autonomous.Init.HardwareAtlas;

@TeleOp(name= "TeleExample", group= "Pushbot")
public class TeleExample extends OpMode {

    private DcMotor left;
    private DcMotor right;

    @Override
    public void init() {
        // The names of the motors for the config on the phone
        left = hardwareMap.dcMotor.get("LeftMotor");
        right = hardwareMap.dcMotor.get("RightMotor");
    }

    @Override
    public void loop() {

        // The 0.1 is the 'deadzone'
        if(gamepad1.left_stick_y >= 0.1 || gamepad1.left_stick_y <= -1) {
            left.setPower(gamepad1.left_stick_y);
            right.setPower(-gamepad1.left_stick_y);
        }

        if(gamepad1.right_stick_x >= 0.1 || gamepad1.right_stick_x <= -1) {
            left.setPower(gamepad1.left_stick_x);
            right.setPower(gamepad1.left_stick_x);
        }
    }
}
