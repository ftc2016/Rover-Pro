package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldDetector;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "YeEt")
public class VishnuAuto extends LinearOpMode
{
    DcMotor MotorFrontY;
    DcMotor MotorFrontX;
    DcMotor MotorBackX;
    DcMotor MotorBackY;

    DcMotor MotorArm;
    DcMotor MotorExtend;
    DcMotor MotorLand;
    
    Servo ServoMarkerArm;
    
    GoldDetector detector = new GoldDetector();


    @Override
    public void runOpMode() throws InterruptedException
    {
        initialize();

        waitForStart();

        while (opModeIsActive())
        {
            robotLanding();

            findCube();

            moveBackward(0.4); // To not touch the marker when we deploy it

            ServoMarkerArm.setPosition(0);
            ServoMarkerArm.setPosition(1);

            detector.disable();
            requestOpModeStop();
            stop();
            break;
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


        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();

        detector.downscale = 0.4;                                                   // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.PERFECT_AREA;         // Can also be PERFECT_AREA
        detector.perfectAreaScorer.perfectArea = 10000;
        detector.maxAreaScorer.weight = .001;

        detector.setSpeed(DogeCV.DetectionSpeed.BALANCED);

        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.0;

        detector.enable();

    }

    public void moveForward(double distance)
    {
        MotorFrontY.setPower(-0.65);
        MotorBackY.setPower(-0.65);

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
        sleep(100);
    }


    public void moveBackward(double distance)
    {
        MotorFrontY.setPower(0.65);
        MotorBackY.setPower(0.65);

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
        sleep(100);
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
        sleep(100);
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
        sleep(100);
    }

    public void robotLanding()
    {
        MotorLand.setPower(1);
        MotorLand.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double dist = 20.5;
        int COUNTS = distanceToCounts(dist);

        MotorLand.setTargetPosition((MotorLand.getCurrentPosition() + (COUNTS)));


        while (opModeIsActive() && MotorLand.isBusy())
        {
            telemetry.addData("Landing Motor Power",MotorLand.getPower());
            telemetry.update();
        }

        telemetry.addData("Robot Landed", null);
        telemetry.update();

        sleep(500);                //it detects the position

        slideLeft(0.5, 0.65);
        moveForward(1.3);
        slideLeft(0.75,0.65);
        sleep(1500);
    }

    public int distanceToCounts(double distance) {
        int rotations = (int) Math.round(distance * 1000);
        return Math.round(rotations);
    }

    public void findCube()
    {
        telemetry.addData("Finding Cube", null);
        telemetry.update();
        sleep(1500);
        if (detector.isFound())                  //checking left
        {
            telemetry.addData("Cube Found Left", null);
            telemetry.update();
            slideLeft(0.5, 0.3);
            moveForward(3.6);
            clockwise(.2, .3);
            moveForward(.7);
        }
        else
            {
                slideRight(1.33, .3);
                sleep(1500);         //checking center
                if (detector.isFound())
                {
                    moveForward(3.5);           //moving to left one
                    telemetry.addData("Cube found center", null);
                    telemetry.update();
                }
                else
                {
                    telemetry.addData("Cube found right", null);
                    telemetry.update();
                    slideRight(1.33,.3);
                    moveForward(3.0);
                    anticlockwise(.3,.7);//moving to right one
                    moveForward(1);
                }
            }

        }

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

            MotorFrontX.setTargetPosition((MotorFrontX.getCurrentPosition() + (COUNTS)));
            MotorBackX.setTargetPosition((MotorBackX.getCurrentPosition() - (COUNTS)));
            MotorFrontY.setTargetPosition(MotorFrontY.getCurrentPosition() + COUNTS);
            MotorBackY.setTargetPosition(MotorBackY.getCurrentPosition() - COUNTS);

            while (opModeIsActive() && MotorBackX.isBusy() && MotorFrontX.isBusy())
            {
                telemetry.addData("Running Motor X Front and Back", "Encoders");
                telemetry.update();
            }
            sleep(100);

        }

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
            sleep(100);

        }


        /*else
            {
            slideLeft((1500));          //checking left
            if (detector.isFound())
            {
                telemetry.addData("Cube Found Left", null);
                telemetry.update();
                moveForward((1800));
            }
            else                                //checking right
                {
                    telemetry.addData("Cube is on the Right", null);
                    telemetry.update();
                    slideRight(3000);
                    moveForward((1500));
                }*/
        }

