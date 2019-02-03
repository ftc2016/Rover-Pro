package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import static java.lang.Math.max;
import static java.lang.Math.min;

/**
 * TeleOp
 */
@TeleOp(name = "TeleOp Qualifiers")
public class RTeleOPOmni extends OpMode
{
    //Initializing all necessary variables
    float y;
    float x;
    boolean land;
    boolean raise;
    float des_arm_rotation;
    float des_arm_extension;
    float armRotation_posError;
    float armExtension_posError;
    float landingArm_PosError;
    float des_landingArm_position;
    float extend;

    int extendMaxPosition;
    int extendMinPosition;

    //Naming all necessary motors, servos and sensors
    DcMotor MotorFrontY;
    DcMotor MotorFrontX;
    DcMotor MotorBackX;
    DcMotor MotorBackY;

    DcMotor MotorArm;
    DcMotor MotorExtend;
    DcMotor MotorLand;
    DcMotor MotorSweep;

    Servo markerArm;

    //This is where all the variables used in the main program are initialized
    @Override
    public void init()
    {

        //Assigning gamepad values to hardware parts
        y = gamepad1.left_stick_y;
        x = gamepad1.right_stick_x;
        land = gamepad2.a;
        raise = gamepad2.b;
        des_arm_rotation = 0;
        des_arm_extension = 0;
        armRotation_posError = 0;
        armExtension_posError = 0;
        landingArm_PosError = 0;
        des_landingArm_position = 0;


        // Defining all the hardware parts
        MotorFrontX = hardwareMap.dcMotor.get("fx");
        MotorBackX = hardwareMap.dcMotor.get("bx");
        MotorBackY = hardwareMap.dcMotor.get("by");
        MotorFrontY = hardwareMap.dcMotor.get("fy");

        MotorExtend = hardwareMap.dcMotor.get("extend");
        MotorArm = hardwareMap.dcMotor.get("arm");
        MotorLand = hardwareMap.dcMotor.get("land");
        MotorSweep = hardwareMap.dcMotor.get("sweep");

        markerArm = hardwareMap.servo.get("deploy");
        markerArm.setDirection(Servo.Direction.FORWARD);

        //Setting the direction of each motor
        MotorFrontX.setDirection(DcMotorSimple.Direction.REVERSE);                                  //alternating between forward and reverse depending on motor placement
        MotorBackX.setDirection(DcMotorSimple.Direction.FORWARD);
        MotorFrontY.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorBackY.setDirection(DcMotorSimple.Direction.FORWARD);

        MotorExtend.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorArm.setDirection(DcMotorSimple.Direction.FORWARD);
        MotorLand.setDirection(DcMotorSimple.Direction.FORWARD);
        MotorSweep.setDirection(DcMotorSimple.Direction.REVERSE);

        MotorSweep.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //Configuring encoders to motors
        MotorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        MotorLand.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorLand.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        MotorExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        extendMinPosition = 600;
        extendMaxPosition = 7600;

    }

    //what is tis
    @Override
    public void loop()
    {
        // initializing wheel variables
        float powerXWheels = 0;
        float powerYWheels = 0;

        //assigning controller values to variable to pass to the motor
        extend = gamepad2.right_stick_y;

        // Handle regular movement
        powerYWheels += gamepad1.left_stick_y;

        // Handle sliding movement
        powerXWheels += gamepad1.right_stick_x;


        // Handle turning movement
        double maxX = (double) powerXWheels;
        double maxY = (double) powerYWheels;


        MotorBackX.setPower((Math.abs(maxX)*maxX));
        MotorFrontX.setPower((Math.abs(maxX)*maxX));

        MotorBackY.setPower((Math.abs(maxY)*maxY));
        MotorFrontY.setPower((Math.abs(maxY)*maxY));

        //Turning clockwise
        if (gamepad1.right_bumper)
        {
            MotorFrontX.setPower(0.5);
            MotorFrontY.setPower(0.5);
            MotorBackX.setPower(-0.5);
            MotorBackY.setPower(-0.5);
        }

        //Turning anticlockwise
        if (gamepad1.left_bumper)
        {
            MotorFrontX.setPower(-0.5);
            MotorFrontY.setPower(-0.5);
            MotorBackX.setPower(0.5);
            MotorBackY.setPower(0.5);
        }

        telemetry.addData("Land value", MotorLand.getCurrentPosition());
        telemetry.update();

        //arm extension starts
        // A position based loop. Helps us to counteract the effect of gravity on arm extension
        des_arm_extension -= 40 * gamepad2.right_stick_y;    //20
        des_arm_extension = (max(min(des_arm_extension, extendMaxPosition),extendMinPosition-100));
        if(gamepad2.a)
            des_arm_extension = 6700;
        armExtension_posError = des_arm_extension - -1*MotorExtend.getCurrentPosition();
        armExtension_posError = (float)(min(max(.00125 * armExtension_posError, -1.0), 1.0));
        MotorExtend.setPower(-armExtension_posError);
        //arm extension control over

        //arm rotation starts
        //A position based loop. Helps us to counteract the effect the effect of gravity on arm rotation
        des_arm_rotation += 25 * gamepad2.left_stick_y;     //20                                        //Accumulator. How much we scale the commands from the controller. Speed
        des_arm_rotation = (float) (min(max(des_arm_rotation, -350.0), 2800.0)); //1150
        armRotation_posError = des_arm_rotation - MotorArm.getCurrentPosition();
        armRotation_posError = (float) (min(max(.0025 * armRotation_posError, -1.0), 1.0));         //Sensitivity. How much torque per encoder count
        MotorArm.setPower(armRotation_posError);
        //Arm rotation over


        //raising and lowering the landing arm
        if (gamepad2.dpad_down)
        {
            MotorLand.setPower(-1.0);
        }

        if (gamepad2.dpad_up)
        {
                MotorLand.setPower(1.0);
        }

        if(!gamepad2.dpad_up && !gamepad2.dpad_down)
        {
            MotorLand.setPower(0);
        }
        //end of raising and landing arm code

        //setting markerArm to -1 all the time
        markerArm.setPosition(-1);


        //Controls for the sweeper
        if(gamepad2.right_bumper)
        {
            MotorSweep.setPower(0.55);
        }
        else if(gamepad2.left_bumper)
        {
            MotorSweep.setPower(-0.55);
        }
        else if(!gamepad2.right_bumper)
            {
                MotorSweep.setPower(0);
            }
    }

    @Override
    public void stop()
    {
    }
}