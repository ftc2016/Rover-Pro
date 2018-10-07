package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;


import static android.os.SystemClock.sleep;



/**
 * TeleOp
 */
@TeleOp(name = "Test Mode")
public class TeleOPTest extends OpMode {
    //Initialising all necessary variables


    DcMotor MotorArm;
    DcMotor MotorExtend;


    @Override
    public void init() {
        telemetry.addData("Initialised", "nothing");
        telemetry.update();

        // defining all the hardware

        MotorExtend = hardwareMap.dcMotor.get("extend");
        MotorArm = hardwareMap.dcMotor.get("arm");


        MotorExtend.setDirection(DcMotorSimple.Direction.FORWARD);
        MotorArm.setDirection(DcMotorSimple.Direction.FORWARD);
    }


    @Override
    public void loop() {
        //telemetry.addData("Enters loop to define power" ,"nothing");
        //telemetry.update();


        float extend = gamepad2.right_stick_y;
        float turn = gamepad2.left_stick_y;

        // Reset variables
        float powerArmExtend = 0;
        float powerTurnArm = 0;



        // Handle extending arm linear slide
        powerArmExtend += extend;

        //Handle turning the arm
        powerTurnArm += turn;

        // Handle turning movement
        double powerturn = (double) powerTurnArm;
        double powerextend = (double) powerArmExtend;

        MotorArm.setPower(powerturn);
        telemetry.addData("Turn Power", powerturn);
        telemetry.update();

        MotorExtend.setPower(powerextend);
        telemetry.addData("Extend Power", powerextend);
        telemetry.update();


        //Gamepad 2 Controls


        // Here you set the motors' power to their respected power double.
        // start of controller movements

    }
}


/*


    public int distanceToCounts(double rotations1){
        int rotations = (int) Math.round (rotations1 * 100);
        return Math.round(rotations);
    }


    public void moveForwardOdometry(double power, int length) {
        MotorFrontY.setPower(-power);
        MotorBackY.setPower(power);
        sleep(length);

    }

    public void moveBackwardOdometry(double power, int length){
        MotorFrontY.setPower(power);;
        MotorBackY.setPower(-power);
        sleep(length);

    }

    public void moveLeftOdometry(double power, int length){
        MotorFrontX.setPower(-power);
        MotorBackX.setPower(power);
        sleep(length);
    }

    public void moveRightOdometry(double power, int length){
        MotorFrontX.setPower(power);
        MotorBackX.setPower(-power);
        sleep(length);
    }



    @Override
    public void stop() {
    }
    double scaleInput(double dVal) {
        double[] scaleArray = {0.0, 0.04, 0.08, 0.9, 0.11, 0.14, 0.17, 0.23, 0.29, 0.35, 0.42, 0.49, 0.59, 0.71, 0.84, 0.99, 1.00};
        int index = (int) (dVal * 16.0);
        if (index < 0) {
            index = -index;
        }
        if (index > 16) {
            index = 16;
        }
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }
        return dScale;
    }
}
*/
