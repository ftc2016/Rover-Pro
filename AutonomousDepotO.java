package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static java.lang.Math.abs;
import static java.lang.Math.max;
import static java.lang.Math.min;

/** This program is run if we start on facing the depot. It helps us centre based on the starting position
 * I am using @param BNO055IMU imu which is the inbuilt Gyroscope in the REV hub.*/

@Autonomous(name = "Scrimmage 4 : Depot (Oppo Crater)", group = "Autonomous Scrimmage")
public class AutonomousDepotO extends LinearOpMode
{
    DcMotor MotorFrontY;
    DcMotor MotorFrontX;
    DcMotor MotorBackX;
    DcMotor MotorBackY;

    DcMotor MotorArm;
    DcMotor MotorExtend;
    DcMotor MotorLand;

    BNO055IMU imu;

    Orientation angles;

    Servo ServoMarkerArm;

    GoldDetector detector = new GoldDetector();

    @Override
    public void runOpMode()
    {
        initialize();
        initDetector();
        waitForStart();

        if (opModeIsActive())
        {
            detector.enable();

            robotLanding();

            findCube();

            detector.disable();
            stop();
        }

    }

    public void initialize()
    {

        MotorFrontX = hardwareMap.dcMotor.get("fx");
        MotorBackX = hardwareMap.dcMotor.get("bx");
        MotorBackY = hardwareMap.dcMotor.get("by");
        MotorFrontY = hardwareMap.dcMotor.get("fy");

        MotorExtend = hardwareMap.dcMotor.get("extend");
        MotorArm = hardwareMap.dcMotor.get("arm");
        MotorLand = hardwareMap.dcMotor.get("land");

        ServoMarkerArm = hardwareMap.servo.get("deploy");

        MotorFrontX.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorBackX.setDirection(DcMotorSimple.Direction.FORWARD);
        MotorFrontY.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorBackY.setDirection(DcMotorSimple.Direction.FORWARD);

        ServoMarkerArm.setDirection(Servo.Direction.FORWARD);

        MotorFrontX.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorFrontY.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorBackX.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorBackY.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        MotorExtend.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorArm.setDirection(DcMotorSimple.Direction.FORWARD);
        MotorLand.setDirection(DcMotorSimple.Direction.FORWARD);

        MotorExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        MotorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        MotorLand.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorLand.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    void initDetector()
    {
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();

        detector.downscale = 0.4;                                                   // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.PERFECT_AREA;         // Can also be PERFECT_AREA
        detector.perfectAreaScorer.perfectArea = 10000;
        detector.maxAreaScorer.weight = .001;

        detector.setSpeed(DogeCV.DetectionSpeed.BALANCED);

        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.0;
    }


    public void moveForward(double distance, double power)
    {
        ////movementMotors()();
        MotorFrontY.setPower(-power);
        MotorBackY.setPower(-power);

        MotorFrontY.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorBackY.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Calling count", "");
        telemetry.update();

        int COUNTS = distanceToCounts(distance);

        telemetry.addData("Counts", COUNTS);
        telemetry.update();

        telemetry.addData("MotorFrontY", MotorFrontY.getCurrentPosition());
        telemetry.addData("MotorBackY", MotorBackY.getCurrentPosition());
        telemetry.update();

        MotorFrontY.setTargetPosition((MotorFrontY.getCurrentPosition() - (COUNTS)));
        MotorBackY.setTargetPosition((MotorBackY.getCurrentPosition() - (COUNTS)));


        while (opModeIsActive() && MotorBackY.isBusy() && MotorFrontY.isBusy())
        {
            telemetry.addData("Running motor Y front and back", "Encoders");
            telemetry.update();
        }
    }


    public void moveBackward(double distance, double power)
    {
        MotorFrontY.setPower(power);
        MotorBackY.setPower(power);

        MotorFrontY.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorBackY.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int COUNTS = distanceToCounts (distance);

        MotorFrontY.setTargetPosition((MotorFrontY.getCurrentPosition() + (COUNTS)));
        MotorBackY.setTargetPosition((MotorBackY.getCurrentPosition() + (COUNTS)));


        while (opModeIsActive() && MotorBackY.isBusy() && MotorFrontY.isBusy()) {
            telemetry.addData("Running motor Y front and Back", "Encoders");
            telemetry.update();
        }
    }


    public void slideRight(double distance, double power)
    {
        MotorFrontX.setPower(power);
        MotorBackX.setPower(power);

        MotorFrontX.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorBackX.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int COUNTS = distanceToCounts(distance);

        MotorFrontX.setTargetPosition((MotorFrontX.getCurrentPosition() + (COUNTS)));
        MotorBackX.setTargetPosition((MotorBackX.getCurrentPosition() + (COUNTS)));

        while (opModeIsActive() && MotorBackX.isBusy() && MotorFrontX.isBusy())
        {
            telemetry.addData("Running motor X front and back", "Encoders");
            telemetry.update();
        }
    }

    public void slideLeft (double distance, double power)
    {

        MotorFrontX.setPower(-power);
        MotorBackX.setPower(-power);


        telemetry.addData("Reached SlideLeft, Ready to  setMode", "none");
        telemetry.update();

        MotorFrontX.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addData("Front Motor X mode set", "Nothing");
        telemetry.update();

        MotorBackX.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Back Motor X", "");
        telemetry.update();

        int COUNTS = distanceToCounts(distance);

        MotorFrontX.setTargetPosition((MotorFrontX.getCurrentPosition() - (COUNTS)));
        MotorBackX.setTargetPosition((MotorBackX.getCurrentPosition() - (COUNTS)));

        while (opModeIsActive() && MotorBackX.isBusy() && MotorFrontX.isBusy())
        {
            telemetry.addData("Running Motor X Front and Back", "Encoders");
            telemetry.update();
        }
    }




    void robotLanding()
    {
        MotorLand.setPower(1);
        MotorLand.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        MotorLand.setTargetPosition(12600);

        while (opModeIsActive() && MotorLand.isBusy())
        {
            telemetry.addData("Landing Motor Power",MotorLand.getPower());
            telemetry.update();
        }

        telemetry.addData("Robot Landed", null);
        telemetry.update();

        slideLeft(0.5, 0.65);
        anticlockwise(0.1,0.1);
        moveForward(1.4, 0.8);
        slideLeft(0.50,0.65);
    }

    public int distanceToCounts(double distance)
    {
        int rotations = (int) Math.round(distance * 1000);
        return Math.round(rotations);
    }

    public void findCube()
    {
        telemetry.addData("Finding Cube", null);
        telemetry.update();
        sleep(500);
        if (detector.isFound())                  //checking left
        {
            goToCraterLeft();
        }
        else
        {
            slideRight(1.33, .3);
            sleep(500);                    //checking center
            if (detector.isFound())
            {
                goToCraterCenter();
            }
            else
            {
                goToCraterRight();
            }
        }

    }

    public void clockwise(double power, double distance)
    {
        //movementMotors()();
        MotorBackY.setPower(power);
        MotorBackX.setPower(power);
        MotorFrontX.setPower(power);
        MotorFrontY.setPower(power);

        telemetry.addData("Reached SlideLeft, Ready to  setMode", "none");
        telemetry.update();

        MotorFrontX.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addData("Front Motor X mode set", "Nothing");
        telemetry.update();

        MotorBackX.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Back Motor X", "");
        telemetry.update();

        int COUNTS = distanceToCounts(distance);

        MotorFrontX.setTargetPosition(MotorFrontX.getCurrentPosition() + (COUNTS));
        MotorBackX.setTargetPosition(MotorBackX.getCurrentPosition() - (COUNTS));
        MotorFrontY.setTargetPosition(MotorFrontY.getCurrentPosition() + COUNTS);
        MotorBackY.setTargetPosition(MotorBackY.getCurrentPosition() - COUNTS);

        while (opModeIsActive() && MotorBackX.isBusy() && MotorFrontX.isBusy())
        {
            telemetry.addData("Running Motor X Front and Back", "Encoders");
            telemetry.update();
        }

    }

    public void anticlockwise(double power, double distance)
    {
        //movementMotors()();
        MotorBackY.setPower(power);
        MotorBackX.setPower(power);
        MotorFrontX.setPower(power);
        MotorFrontY.setPower(power);

        telemetry.addData("Reached SlideLeft, Ready to  setMode", "none");
        telemetry.update();

        MotorFrontX.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addData("Front Motor X mode set", "Nothing");
        telemetry.update();

        MotorBackX.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Back Motor X", "");
        telemetry.update();

        int COUNTS = distanceToCounts(distance);

        MotorFrontX.setTargetPosition((MotorFrontX.getCurrentPosition() - (COUNTS)));
        MotorBackX.setTargetPosition((MotorBackX.getCurrentPosition() + (COUNTS)));
        MotorFrontY.setTargetPosition(MotorFrontY.getCurrentPosition() - COUNTS);
        MotorBackY.setTargetPosition(MotorBackY.getCurrentPosition() + COUNTS);

        while (opModeIsActive() && MotorBackX.isBusy() && MotorFrontX.isBusy())
        {
            telemetry.addData("Running Motor X Front and Back", "Encoders");
            telemetry.update();
        }

    }

    void goToCraterRight()
    {
        telemetry.addData("Side Chosen", "Right");
        telemetry.update();
        slideRight(1.33,.5);
        moveForward(3.1, 0.75);
        anticlockwise(1,0.85);
        slideRight(1,0.25);
        moveForward(1,1);
        ServoMarkerArm.setPosition(1);
        sleep(1500);
        moveBackward(7, 1.0);
        ServoMarkerArm.setPosition(-1);
    }

    void goToCraterCenter()
    {
        telemetry.addData("Side Chosen", "Center");
        telemetry.update();
        slideLeft(0.2, 0.5);
        moveForward(3.75, 1.0);
        ServoMarkerArm.setPosition(1.0);
        sleep(500);
        clockwise(0.8, 0.6);
        slideLeft(1.0,0.5);
        moveBackward(1, 1.0);
        slideLeft(1,1);

        moveBackward(6.5, 1.0);
        ServoMarkerArm.setPosition(-1);
        clockwise(0.8, 0.5);
        moveBackward(0.2, 0.1);
    }

    void goToCraterLeft()
    {
        telemetry.addData("Side Chosen", "Left");
        telemetry.update();
        slideLeft(0.5, 0.8);
        moveForward(3.2, 1);
        clockwise(.4, .6);
        moveForward(1, 1);
        ServoMarkerArm.setPosition(1);
        sleep(750);
        slideLeft(0.5, 1);
        moveBackward(6.0, 1);
        ServoMarkerArm.setPosition(-1);
        anticlockwise(0.5, 0.5);
    }
}