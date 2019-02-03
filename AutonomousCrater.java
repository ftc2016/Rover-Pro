package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

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

@Autonomous(name = "Scrimmage 4 : Crater", group ="Autonomous Scrimmage" )
public class AutonomousCrater extends LinearOpMode {
    BNO055IMU imu;

    Orientation angles;

    //Naming all necessary motors, servos and sensors
    DcMotor MotorFrontY;
    DcMotor MotorFrontX;
    DcMotor MotorBackX;
    DcMotor MotorBackY;

    DcMotor MotorArm;
    DcMotor MotorExtend;
    DcMotor MotorLand;

    Servo ServoMarkerArm;
    GoldDetector detector = new GoldDetector();

    //This is what the robot runs
    public void runOpMode()
    {
        initialize();

        waitForStart();

        if(opModeIsActive())
        {
            detector.enable();

            robotLanding();

            findCube();
            detector.disable();
        }
    }

    //This is where all the variables used in the main program are initialized
    public void initialize()
    {
        //initGyro();
        initDetector();

        // Defining all the hardware parts
        MotorFrontX = hardwareMap.dcMotor.get("fx");
        MotorBackX = hardwareMap.dcMotor.get("bx");
        MotorBackY = hardwareMap.dcMotor.get("by");
        MotorFrontY = hardwareMap.dcMotor.get("fy");

        MotorExtend = hardwareMap.dcMotor.get("extend");
        MotorArm = hardwareMap.dcMotor.get("arm");
        MotorLand = hardwareMap.dcMotor.get("land");

        ServoMarkerArm = hardwareMap.servo.get("deploy");

        //Setting the direction of each motor and servo
        MotorFrontX.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorBackX.setDirection(DcMotorSimple.Direction.FORWARD);
        MotorFrontY.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorBackY.setDirection(DcMotorSimple.Direction.FORWARD);

        ServoMarkerArm.setDirection(Servo.Direction.FORWARD);

        //Configuring encoders to motors
        MotorFrontX.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorFrontY.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorBackX.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorBackY.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        MotorExtend.setDirection(DcMotorSimple.Direction.REVERSE);
        MotorArm.setDirection(DcMotorSimple.Direction.FORWARD);
        MotorLand.setDirection(DcMotorSimple.Direction.FORWARD);

        //Resetting all encoder values to 0
        MotorExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        MotorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        MotorLand.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MotorLand.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //This is where DogeCV is initialized
    void initDetector()
    {

        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();

        detector.downscale = 5;                                                   //0.4  How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.PERFECT_AREA;         // Can also be PERFECT_AREA
        detector.perfectAreaScorer.perfectArea = 10000;
        detector.maxAreaScorer.weight = .001;

        detector.setSpeed(DogeCV.DetectionSpeed.BALANCED);

        detector.ratioScorer.weight = 5;        //5
        detector.ratioScorer.perfectRatio = 1.0;    //1
    }

    //This is the encoder method for the robot to move forward
    public void moveForward(double distance, double power)
    {
        MotorFrontY.setPower(-power);
        MotorBackY.setPower(-power);

        telemetry.addData("Calling count", "");
        telemetry.update();

        int COUNTS = distanceToCounts(distance);

        telemetry.addData("Counts", COUNTS);
        telemetry.update();

        MotorFrontY.setTargetPosition((MotorFrontY.getCurrentPosition() - (COUNTS)));
        MotorBackY.setTargetPosition((MotorBackY.getCurrentPosition() - (COUNTS)));

        //moved from line 3 of method
        MotorFrontY.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorBackY.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        while (opModeIsActive() && MotorBackY.isBusy() && MotorFrontY.isBusy())
        {
            telemetry.addData("Running motor Y front and back", "Encoders");
            telemetry.update();
        }
    }

    //This is the encoder method for the robot to move backward
    public void moveBackward(double distance, double power)
    {
        MotorFrontY.setPower(power);
        MotorBackY.setPower(power);

        MotorFrontY.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorBackY.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int COUNTS = distanceToCounts (distance);

        MotorFrontY.setTargetPosition((MotorFrontY.getCurrentPosition() + (COUNTS)));
        MotorBackY.setTargetPosition((MotorBackY.getCurrentPosition() + (COUNTS)));


        while (opModeIsActive() && MotorBackY.isBusy() && MotorFrontY.isBusy())
        {
            telemetry.addData("Running motor Y front and Back", "Encoders");
            telemetry.update();
        }
    }

    //This is the encoder method for the robot to slide right
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

    //This is the encoder program for the robot to slide left
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

    //This is the method for the robot to turn clockwise
    public void clockwise(double power, double distance)
    {
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

    //This is the method for the robot to turn anticlockwise
    public void anticlockwise(double power, double distance)
    {
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

    public int distanceToCounts(double distance)
    {
        int rotations = (int) Math.round(distance * 1000);
        return Math.round(rotations);
    }

    //This method defines robot movement while it is performing the landing mission
    public void robotLanding()
    {
        MotorLand.setPower(1);
        MotorLand.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        MotorLand.setTargetPosition(12600);

        while (opModeIsActive() && MotorLand.isBusy()) {
            telemetry.addData("Landing Motor Power", MotorLand.getPower());
            telemetry.update();
        }

        telemetry.addData("Robot Landed", null);
        telemetry.update();

        slideLeft(0.5, 0.8);
        //initGyro();
        //rotationCorrection();
        anticlockwise(0.1, 0.1);
        moveForward(1.65, 1);
        slideLeft(0.50, 0.8);
    }

    //This method defines robot movement while performing the sampling mission
    public void findCube()
    {
        telemetry.addData("Finding Cube", null);
        telemetry.update();
        sleep(500);
        if (detector.isFound())                  //checking left
        {
            goToDepotLeft();
        }
        else
        {
            slideRight(1.33, .7);
            sleep(500);                    //checking center
            if (detector.isFound())
            {
                goToDepotCenter();
            }
            else
            {
                goToDepotRight();
            }
        }

    }

    //If the robot detects the gold mineral at the left position, it will run this method
    void goToDepotLeft()
    {

        telemetry.addData("Side Chosen", "Left");
        telemetry.update();
        slideLeft(0.5, 0.5);
        moveForward(1.25, 0.8);
        moveBackward(1.25, 0.8);
        slideLeft(2.75, .8);
        anticlockwise(0.8, 1.8);
        slideRight(1.4, .8);
        moveForward(3.5, .8);
        ServoMarkerArm.setPosition(1);
        sleep(1500);
        moveBackward(6, .9);
        ServoMarkerArm.setPosition(-1);
        anticlockwise(.75, .4);
        moveBackward(0.99, 0.5);

    }

    //If the robot detects the gold mineral at the center position, it will run this method
    void goToDepotCenter()
    {
        telemetry.addData("Side Chosen", "Center");
        telemetry.update();
        slideLeft(0.2,0.5);
        moveForward(1.25 ,.8);
        moveBackward(1.25, .8);
        slideLeft(3.5, .8);
        anticlockwise(.8, 1.85);
        slideRight(1.4, .8);
        moveForward(5, .8);
        ServoMarkerArm.setPosition(1.0);
        sleep(1500);
        moveBackward(6, .9);
        ServoMarkerArm.setPosition(-1);
        anticlockwise(.75, 0.2);
        moveBackward(0.6, 0.5);

    }

    //If the robot detects the gold mineral at the right position, it will run this method
    void goToDepotRight()
    {
        telemetry.addData("Side Chosen", "Right");
        telemetry.update();

        slideRight(1.3,.5);
        moveForward(1.25, .8);
        moveBackward(1.25, .8);
        slideLeft(4.0, .8);
        anticlockwise(.8,2);
        slideRight(1.6, .8);
        moveForward(5.0, .8);
        ServoMarkerArm.setPosition(1);
        sleep(1500);
        moveBackward(6, .9);
        ServoMarkerArm.setPosition(-1);
        MotorBackY.setTargetPosition(-1257);
        anticlockwise(.75, .5);
        moveBackward(0.99,0.5);
    }
}