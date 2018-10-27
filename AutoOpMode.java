package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "Auto: Basic", group = "Autonomous")
public class AutoOpMode extends LinearOpMode {

    DcMotor MotorFrontX;
    DcMotor MotorFrontY;
    DcMotor MotorBackX;
    DcMotor MotorBackY;

    DcMotor MotorArm;
    DcMotor MotorExtend;
    DcMotor MotorLand;
    private SamplingOrderDetector detector;

    char goldPosition;

    @Override
    public void runOpMode() throws InterruptedException
    {

        initialize();

        waitForStart();

        //start opMode loop. Main Code Goes Here.
        while (opModeIsActive())
        {

            goldPosition = getGoldPosition();

            telemetry.addData("Outside Gold Position : ", goldPosition);
            telemetry.update();
            sleep(1000);

            robotLanding();
            telemetry.addData("After Landing", null);
            telemetry.update();
            sleep(1000);//it detects the position

            slideLeft(0.5);
            moveForward(1.5);
            slideRight(0.5);

            double d = 1.2;
            if(goldPosition == 'c'){
                moveForward(d);
                moveForward(1.0);
            }else if(goldPosition == 'l')
            {
                slideLeft(d);
                moveForward(1.0);
            } else if(goldPosition == 'r'){
                slideRight(d);
                moveForward(1.0);
            }
            else{
                telemetry.addData("Nothing Matched, Moving forward", null);
                moveForward(d);
            }



            //slideRight(5);
            setPower(0);
            detector.disable();
            stop();
            requestOpModeStop();
            break;
        }
    }

    protected void initialize()
    {
        MotorFrontX = (DcMotor) hardwareMap.dcMotor.get("fx");
        MotorFrontY = (DcMotor) hardwareMap.dcMotor.get("fy");
        MotorBackX = (DcMotor) hardwareMap.dcMotor.get("bx");
        MotorBackY = (DcMotor) hardwareMap.dcMotor.get("by");

        MotorExtend = hardwareMap.dcMotor.get("extend");
        MotorArm = hardwareMap.dcMotor.get("arm");
        MotorLand = hardwareMap.dcMotor.get("land");

        // Reverse the motor that runs backwards when connected directly to the battery
        MotorFrontX.setDirection(DcMotor.Direction.REVERSE);
        MotorFrontY.setDirection(DcMotor.Direction.REVERSE);
        MotorBackX.setDirection(DcMotor.Direction.FORWARD);
        MotorBackY.setDirection(DcMotor.Direction.FORWARD);

        MotorExtend.setDirection(DcMotor.Direction.REVERSE);
        MotorArm.setDirection(DcMotor.Direction.REVERSE);
        MotorLand.setDirection(DcMotor.Direction.FORWARD);

        //Motors set to run with encoders
        MotorBackX.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorBackY.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorFrontX.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorFrontY.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        MotorExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorLand.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        MotorLand.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("Status", "DogeCV 2018.0 - Sampling Order Example");
        telemetry.update();

        detector = new SamplingOrderDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();

        detector.downscale = 1; // How much to downscale the input frames

        // Optional Tuning
        detector.areaScoringMethod = DogeCV.AreaScoringMethod.PERFECT_AREA; // Can also be PERFECT_AREA
        detector.perfectAreaScorer.perfectArea = 10000;
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = .001;

        detector.setSpeed(DogeCV.DetectionSpeed.BALANCED);

        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.0;

        detector.enable();
        telemetry.addData("Initialise successful ", null);
        telemetry.update();
     //   detector.enable();
        sleep(1000);
        }

    public void robotLanding (){

        //Landing
        MotorLand.setPower(1);

        double dist = 20.5;
        int COUNTS = distanceToCounts (dist);

        telemetry.addData("Counts", COUNTS);
        telemetry.update();
        MotorLand.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        MotorLand.setTargetPosition((MotorLand.getCurrentPosition() + (COUNTS)));

        telemetry.addData("Robot Landing", (MotorLand.getCurrentPosition() + (COUNTS)));
        telemetry.update();


        while (opModeIsActive() && MotorLand.isBusy())
        {
            telemetry.addData("Robot Landing", "Encoders");
            telemetry.update();
        }

        telemetry.addData(" After Robot Landing ", (MotorLand.getCurrentPosition() + (COUNTS)));
        telemetry.update();

        //setPower(0);

        //Release handle

        //MotorLand.setTargetPosition((MotorLand.getCurrentPosition() - (COUNTS)));

        /*while (opModeIsActive() && MotorLand.isBusy())
        {
            telemetry.addData("Taking Landing arm down", "Encoders");
            telemetry.update();
        }*/

        //Come back to center position

        /*setPower(0);*/
    }

    public char getGoldPosition() {
        char goldPosition;
        int center = 0;
        int left = 0;
        int right = 0;
        int uk = 0;

        try {
            for (int i = 0; i < 100; i++)
            {
                if ((detector.getCurrentOrder().toString()).equalsIgnoreCase("LEFT")) {
                    left++;
                } else if ((detector.getCurrentOrder().toString()).equalsIgnoreCase("RIGHT")) {
                    right++;
                } else if (detector.getCurrentOrder().toString().equalsIgnoreCase("CENTER")) {
                    center++;
                } else                                                                          //ranai changed this 10/26
                    uk++;
            }
            telemetry.addData("Order in gold posn method", detector.getCurrentOrder().toString());
            telemetry.update();
            sleep(1000);
        }
        catch (Exception e) {
            telemetry.addData("Exception: ", e);
        }

        if(center>left && center>right)
            goldPosition = 'c';
        else if(left>center && left>right)
            goldPosition = 'l';
        else if(right>center && right>left)
            goldPosition = 'r';
        else
        {
            goldPosition = 'n';
            MotorBackX.setPower(0);
            MotorFrontX.setPower(0);
            MotorBackY.setPower(0);
            MotorFrontY.setPower(0);
        }

        telemetry.addData("Gold Position: ", goldPosition);
        telemetry.addData("Center: ", center);
        telemetry.addData("left: ", left);
        telemetry.addData("Right: ", right);
        telemetry.addData("Unknown /n", uk);
        telemetry.addData("Exiting getGold Posn method",null);
        telemetry.update();
        sleep(10000);

        return goldPosition;
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
        setPower(0);
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
        setPower(0);
        sleep(100);
    }


    public void slideRight(double distance)
    {
        MotorFrontX.setPower(0.65);
        MotorBackX.setPower(0.65);

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
        setPower(0);
        sleep(100);
    }

    public void slideLeft (double distance)
    {
        MotorFrontX.setPower(-0.65);
        MotorBackX.setPower(-0.65);


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
        setPower(0);
        sleep(100);
    }

    public void turnClockwise(double distance)
    {

        MotorFrontX.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorFrontY.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorBackX.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorBackY.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int COUNTS = distanceToCounts(distance);

        MotorFrontY.setTargetPosition((MotorFrontY.getCurrentPosition() + (COUNTS)));
        MotorBackY.setTargetPosition((MotorBackY.getCurrentPosition() - (COUNTS)));
        MotorFrontX.setTargetPosition((MotorFrontX.getCurrentPosition() + (COUNTS)));
        MotorBackX.setTargetPosition((MotorBackX.getCurrentPosition() - (COUNTS)));

        MotorFrontY.setPower(0.35);
        MotorBackY.setPower(-0.35);
        MotorFrontX.setPower(-0.35);
        MotorBackX.setPower(0.35);


        while (opModeIsActive() && MotorBackY.isBusy() && MotorBackX.isBusy() &&
                MotorFrontX.isBusy() && MotorFrontY.isBusy())
        {
            telemetry.addData("Running", "Encoders");
            telemetry.update();
        }
        setPower(0);
        sleep(100);

    }
    public void turnAntiClockwise(double distance)
    {

        MotorFrontX.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorFrontY.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorBackX.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MotorBackY.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int COUNTS = distanceToCounts(distance);

        MotorFrontY.setTargetPosition((MotorFrontY.getCurrentPosition() - (COUNTS)));
        MotorBackY.setTargetPosition((MotorBackY.getCurrentPosition() + (COUNTS)));
        MotorFrontX.setTargetPosition((MotorFrontX.getCurrentPosition() - (COUNTS)));
        MotorBackX.setTargetPosition((MotorBackX.getCurrentPosition() + (COUNTS)));

        MotorFrontY.setPower(-0.65);
        MotorBackY.setPower(0.65);
        MotorFrontX.setPower(0.65);
        MotorBackX.setPower(-0.65);


        while (opModeIsActive() && MotorBackY.isBusy() && MotorBackX.isBusy() &&
                MotorFrontX.isBusy() && MotorFrontY.isBusy())
        {
            telemetry.addData("Running", "Encoders");
            telemetry.update();
        }
        setPower(0);
        sleep(100);

    }

    public void moveArmForward(double distance)
    {
        MotorArm.setPower(-0.65);

        MotorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Calling count", "");
        telemetry.update();

        int COUNTS = distanceToCounts(distance);

        telemetry.addData("Counts", COUNTS);
        telemetry.update();

        telemetry.addData("MotorArm", MotorArm.getCurrentPosition());
        telemetry.update();

        MotorArm.setTargetPosition((MotorArm.getCurrentPosition() - (COUNTS)));


        while (opModeIsActive() && MotorArm.isBusy())
        {
            telemetry.addData("Running motor arm forward", "Encoders");
            telemetry.update();
        }
        setPower(0);
        sleep(100);
    }

    public void moveArmBackward(double distance)
    {
        MotorArm.setPower(0.65);

        MotorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Calling count", "");
        telemetry.update();

        int COUNTS = distanceToCounts(distance);

        telemetry.addData("Counts", COUNTS);
        telemetry.update();

        telemetry.addData("MotorArm", MotorArm.getCurrentPosition());
        telemetry.update();

        MotorArm.setTargetPosition((MotorArm.getCurrentPosition() + (COUNTS)));


        while (opModeIsActive() && MotorArm.isBusy())
        {
            telemetry.addData("Running motor arm backward", "Encoders");
            telemetry.update();
        }
        setPower(0);
        sleep(100);
    }

    public void extendArmForward(double distance)
    {
        MotorExtend.setPower(0.65);

        MotorExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Calling count", "");
        telemetry.update();

        int COUNTS = distanceToCounts(distance);

        telemetry.addData("Counts", COUNTS);
        telemetry.update();

        telemetry.addData("MotorExtend", MotorArm.getCurrentPosition());
        telemetry.update();

        MotorExtend.setTargetPosition((MotorExtend.getCurrentPosition() + (COUNTS)));


        while (opModeIsActive() && MotorExtend.isBusy())
        {
            telemetry.addData("Running motor extend forward", "Encoders");
            telemetry.update();
        }
        setPower(0);
        sleep(100);
    }

    public void extendArmBackward(double distance)
    {
        MotorExtend.setPower(-0.65);

        MotorExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Calling count", "");
        telemetry.update();

        int COUNTS = distanceToCounts(distance);

        telemetry.addData("Counts", COUNTS);
        telemetry.update();

        telemetry.addData("MotorExtend", MotorArm.getCurrentPosition());
        telemetry.update();

        MotorExtend.setTargetPosition((MotorExtend.getCurrentPosition() - (COUNTS)));


        while (opModeIsActive() && MotorExtend.isBusy())
        {
            telemetry.addData("Running motor extend forward", "Encoders");
            telemetry.update();
        }
        setPower(0);
        sleep(100);
    }


    public void setPower(double power)
    {
        MotorFrontX.setPower(power);
        MotorFrontY.setPower(power);
        MotorBackX.setPower(power);
        MotorBackY.setPower(power);
    }

    public int distanceToCounts(double distance)
    {
        int rotations = (int) Math.round(distance*1000);
        return Math.round(rotations);
    }




    public void placeMarker(){
        extendArmForward(6);
        moveArmForward(.8);
        extendArmBackward(6);

        moveArmBackward(.25);

        telemetry.addData("Marker Placed", "Encoders");
        telemetry.update();

    }

}
