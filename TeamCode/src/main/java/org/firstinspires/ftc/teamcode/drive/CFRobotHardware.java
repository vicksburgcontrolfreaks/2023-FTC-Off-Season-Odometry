/* Copyright (c) 2022 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

/**
 * This file works in conjunction with the External Hardware Class sample called: ConceptExternalHardwareClass.java
 * Please read the explanations in that Sample about how to use this class definition.
 *
 * This file defines a Java Class that performs all the setup and configuration for a sample robot's hardware (motors and sensors).
 * It assumes three motors (left_drive, right_drive and arm) and two servos (left_hand and right_hand)
 *
 * This one file/class can be used by ALL of your OpModes without having to cut & paste the code each time.
 *
 * Where possible, the actual hardware objects are "abstracted" (or hidden) so the OpMode code just makes calls into the class,
 * rather than accessing the internal hardware directly. This is why the objects are declared "private".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with *exactly the same name*.
 *
 * Or.. In OnBot Java, add a new file named RobotHardware.java, drawing from this Sample; select Not an OpMode.
 * Also add a new OpMode, drawing from the Sample ConceptExternalHardwareClass.java; select TeleOp.
 *
 */

/**
 *  This file illustrates the concept of driving an autonomous path based on Gyro heading and encoder counts.
 *  The code is structured as a LinearOpMode
 *
 *  The path to be followed by the robot is built from a series of drive, turn or pause steps.
 *  Each step on the path is defined by a single function call, and these can be strung together in any order.
 *
 *  The code REQUIRES that you have encoders on the drive motors, otherwise you should use: RobotAutoDriveByTime;
 *
 *  This code ALSO requires that you have a BOSCH BNO055 IMU, otherwise you would use: RobotAutoDriveByEncoder;
 *  This IMU is found in REV Control/Expansion Hubs shipped prior to July 2022, and possibly also on later models.
 *  To run as written, the Control/Expansion hub should be mounted horizontally on a flat part of the robot chassis.
 *
 *  This sample requires that the drive Motors have been configured with names : left_drive and right_drive.
 *  It also requires that a positive power command moves both motors forward, and causes the encoders to count UP.
 *  So please verify that both of your motors move the robot forward on the first move.  If not, make the required correction.
 *  See the beginning of runOpMode() to set the FORWARD/REVERSE option for each motor.
 *
 *  This code uses RUN_TO_POSITION mode for driving straight, and RUN_USING_ENCODER mode for turning and holding.
 *  Note: You must call setTargetPosition() at least once before switching to RUN_TO_POSITION mode.
 *
 *  Notes:
 *
 *  All angles are referenced to the coordinate-frame that is set whenever resetHeading() is called.
 *  In this sample, the heading is reset when the Start button is touched on the Driver station.
 *  Note: It would be possible to reset the heading after each move, but this would accumulate steering errors.
 *
 *  The angle of movement/rotation is assumed to be a standardized rotation around the robot Z axis,
 *  which means that a Positive rotation is Counter Clockwise, looking down on the field.
 *  This is consistent with the FTC field coordinate conventions set out in the document:
 *  ftc_app\doc\tutorial\FTC_FieldCoordinateSystemDefinition.pdf
 *
 *  Control Approach.
 *
 *  To reach, or maintain a required heading, this code implements a basic Proportional Controller where:
 *
 *      Steering power = Heading Error * Proportional Gain.
 *
 *      "Heading Error" is calculated by taking the difference between the desired heading and the actual heading,
 *      and then "normalizing" it by converting it to a value in the +/- 180 degree range.
 *
 *      "Proportional Gain" is a constant that YOU choose to set the "strength" of the steering response.
 *
 *  Use Android Studio to Copy this Class, and Paste it into your "TeamCode" folder with a new name.
 *  Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */


public class CFRobotHardware {

    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.
    // Define a constructor that allows the OpMode to pass a reference to itself.
    public CFRobotHardware(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    private DcMotor left_front  = null;
    private DcMotor right_front = null;
    private DcMotor right_rear  = null;
    private DcMotor left_rear   = null;
    DcMotor slide_motor         = null;
    private BNO055IMU imu       = null;      // Control/Expansion Hub IMU
    Servo Back                  = null;
    Servo Front                 = null;

    private double robotHeading  = 0;
    private double headingOffset = 0;
    private double headingError  = 0;

    int slideTopPosition = 3400;
    int slideMiddlePosition = 2450;
    int slideLowPosition = 1400;
    int slideBottomPosition = 50;

    // These variable are declared here (as class members) so they can be updated in various methods,
    // but still be displayed by sendTelemetry()
    private double  targetHeading       = 0;
    private double  driveSpeed          = 0;
    private double  turnSpeed           = 0;
    private double  leftSpeed           = 0;
    private double  rightSpeed          = 0;
    private int     leftFrontTarget     = 0;
    private int     leftRearTarget      = 0;
    private int     rightFrontTarget    = 0;
    private int     rightRearTarget     = 0;
    boolean         bugOut          = false;
    boolean         tryToGetACone   = false;
    int             counter             = 0;
    boolean         START_LEFT;
    double          TURN_SPEED;
    double          DRIVE_SPEED;

    // Define Drive constants.  Make them public so they CAN be used by the calling OpMode
    static final double     COUNTS_PER_MOTOR_REV    = 28.0 ;     // REV HD Hex
    static final double     DRIVE_GEAR_REDUCTION    = 15.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 3.78 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    // These constants define the desired driving/control characteristics
    // They can/should be tweaked to suit the specific robot drive train.
    static final double     HEADING_THRESHOLD       = 1.0 ;    // How close must the heading get to the target before moving to next step.
    // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not correct strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    static final double     P_TURN_GAIN            = 0.02;     // Larger is more responsive, but also less stable
    static final double     P_DRIVE_GAIN           = 0.03;     // Larger is more responsive, but also less stable

    static final double FEET_PER_METER = 3.28084;

    DigitalChannel touch;  // Hardware Device Object

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int LEFT    = 35; // Tag ID from the 36h11 family
    int MIDDLE  = 36;
    int RIGHT   = 37;

    AprilTagDetection tagOfInterest = null;

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     *
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void init()    {
        // Define and Initialize Motors (note: need to use reference to actual OpMode).
        left_front = myOpMode.hardwareMap.get(DcMotor.class, "left_front");
        right_front = myOpMode.hardwareMap.get(DcMotor.class, "right_front");
        left_rear = myOpMode.hardwareMap.get(DcMotor.class, "left_rear");
        right_rear = myOpMode.hardwareMap.get(DcMotor.class, "right_rear");
        slide_motor = myOpMode.hardwareMap.get(DcMotor.class, "slide_motor");
        Back = myOpMode.hardwareMap.get(Servo.class, "Back");
        Front = myOpMode.hardwareMap.get(Servo.class, "Front");
        touch = myOpMode.hardwareMap.get(DigitalChannel.class, "touch");
        touch.setMode(DigitalChannel.Mode.INPUT);

        left_front.setDirection(DcMotor.Direction.FORWARD);
        left_rear.setDirection(DcMotor.Direction.FORWARD);
        right_front.setDirection(DcMotor.Direction.REVERSE);
        right_rear.setDirection(DcMotor.Direction.REVERSE);
        slide_motor.setDirection(DcMotor.Direction.FORWARD);

        slide_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide_motor.setTargetPosition(0);
        slide_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // define initialization values for IMU, and then initialize it.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        imu = myOpMode.hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }

    public void runAprilTag(){
        int cameraMonitorViewId = myOpMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", myOpMode.hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(myOpMode.hardwareMap.get(WebcamName.class, "Webcam 2"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()

        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(1920,1080, OpenCvCameraRotation.UPRIGHT);
            } //640x480, 1280x720, 1024x768, 800x448, 960x720, 960x544, 864x480, 848x480, 800x600, 800x448, 640x360, 352x288, 320x240, 1920x1080

            @Override
            public void onError(int errorCode)
            {

            }
        });

        myOpMode.telemetry.setMsTransmissionInterval(50);


        while (!myOpMode.isStarted() && !myOpMode.isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    myOpMode.telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    myOpMode.telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        myOpMode.telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        myOpMode.telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                myOpMode.telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    myOpMode.telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    myOpMode.telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            myOpMode.telemetry.update();
            myOpMode.sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        camera.closeCameraDevice(); //shut off the camera to preserve battery

        if(tagOfInterest != null)
        {
            myOpMode.telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            myOpMode.telemetry.update();
        }
        else
        {
            myOpMode.telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            myOpMode.telemetry.update();
        }
    }

    public void runWithEncoders(){
        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_rear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_rear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set the encoders for closed loop speed control, and reset the heading.
        left_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_rear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_rear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void floatChassis(){
        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        left_rear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        right_rear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void brakeChassis(){
        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_rear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_rear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void driveStraight(double maxDriveSpeed,
                              double distance,
                              double heading) {

        // Ensure that the opmode is still active
        if (myOpMode.opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int moveCounts = (int)(distance * COUNTS_PER_INCH);
            leftFrontTarget = left_front.getCurrentPosition() + moveCounts;
            leftRearTarget = left_rear.getCurrentPosition() + moveCounts;
            rightFrontTarget = right_front.getCurrentPosition() + moveCounts;
            rightRearTarget = right_rear.getCurrentPosition() + moveCounts;

            // Set Target FIRST, then turn on RUN_TO_POSITION
            left_front.setTargetPosition(leftFrontTarget);
            left_rear.setTargetPosition(leftRearTarget);
            right_front.setTargetPosition(rightFrontTarget);
            right_rear.setTargetPosition(rightRearTarget);

            left_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            left_rear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right_front.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            right_rear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
            // Start driving straight, and then enter the control loop
            maxDriveSpeed = Math.abs(maxDriveSpeed);
            moveRobot(maxDriveSpeed, 0);

            // keep looping while we are still active, and ALL motors are running.
            while (myOpMode.opModeIsActive() &&
                    (left_front.isBusy() && right_front.isBusy() && left_rear.isBusy() && right_rear.isBusy())) {

                // Determine required steering to keep on heading
                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    turnSpeed *= -1.0;

                // Apply the turning correction to the current driving speed.
                moveRobot(driveSpeed, turnSpeed);

                // Display drive status for the driver.
                sendTelemetry(true);
            }

            // Stop all motion & Turn off RUN_TO_POSITION
            moveRobot(0, 0);
            left_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            left_rear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            right_rear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     *  Method to spin on central axis to point in a new direction.
     *  Move will stop if either of these conditions occur:
     *  1) Move gets to the heading (angle)
     *  2) Driver stops the opmode running.
     *
     * @param maxTurnSpeed Desired MAX speed of turn. (range 0 to +1.0)
     * @param heading Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *              If a relative angle is required, add/subtract from current heading.
     */
    public void turnToHeading(double maxTurnSpeed, double heading) {

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN);

        // keep looping while we are still active, and not on heading.
        while (myOpMode.opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0);
    }

    /**
     *  Method to obtain & hold a heading for a finite amount of time
     *  Move will stop once the requested time has elapsed
     *  This function is useful for giving the robot a moment to stabilize it's heading between movements.
     *
     * @param maxTurnSpeed      Maximum differential turn speed (range 0 to +1.0)
     * @param heading    Absolute Heading Angle (in Degrees) relative to last gyro reset.
     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
     *                   If a relative angle is required, add/subtract from current heading.
     * @param holdTime   Length of time (in seconds) to hold the specified heading.
     */
    public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {

        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();

        // keep looping while we have time remaining.
        while (myOpMode.opModeIsActive() && (holdTimer.time() < holdTime)) {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            moveRobot(0, turnSpeed);

            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        moveRobot(0, 0);
    }

    // **********  LOW Level driving functions.  ********************

    /**
     * This method uses a Proportional Controller to determine how much steering correction is required.
     *
     * @param desiredHeading        The desired absolute heading (relative to last heading reset)
     * @param proportionalGain      Gain factor applied to heading error to obtain turning power.
     * @return                      Turning power needed to get to required heading.
     */
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Get the robot heading by applying an offset to the IMU heading
        robotHeading = getRawHeading() - headingOffset;

        // Determine the heading current error
        headingError = targetHeading - robotHeading;

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }

    /**
     * This method takes separate drive (fwd/rev) and turn (right/left) requests,
     * combines them, and applies the appropriate speed commands to the left and right wheel motors.
     * @param drive forward motor speed
     * @param turn  clockwise turning motor speed.
     */
    public void moveRobot(double drive, double turn) {
        driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
        turnSpeed  = turn;      // save this value as a class member so it can be used by telemetry.

        leftSpeed  = drive - turn;
        rightSpeed = drive + turn;

        // Scale speeds down if either one exceeds +/- 1.0;
        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if (max > 1.0)
        {
            leftSpeed /= max;
            rightSpeed /= max;
        }

        left_front.setPower(leftSpeed);
        left_rear.setPower(leftSpeed);
        right_front.setPower(rightSpeed);
        right_rear.setPower(rightSpeed);
    }

    public void Intake(){
        Back.setPosition(0);
        Front.setPosition(1);
    }

    public void Output(){
        Back.setPosition(1);
        Front.setPosition(0);
    }

    public void ServoOff(){
        Back.setPosition(.5);
        Front.setPosition(.5);
    }

    public void getCone() {
        tryToGetACone = true;
        //pick up cone
        slide_motor.setTargetPosition(450);
        Intake();
        ElapsedTime bugoutTimer = new ElapsedTime();
        bugoutTimer.reset();

        while (touch.getState() && tryToGetACone) { // no cone collected yet
            counter += 1;
            // TODO: 12/15/2022
//            if (bugoutTimer.time() >= 1500){ // something went wrong, just go park
//                bugOutAndPark();
//                return;
//            }
            if (counter > 950){ // something went wrong, just go park
                bugOutAndPark();
                return;
            }
        }
        if (!touch.getState()) { // cone activates the switch
            tryToGetACone = false;
            Back.setPosition(.5);
            Front.setPosition(.5);
            slide_motor.setTargetPosition(slideMiddlePosition);//put slide up
            while (slide_motor.getCurrentPosition() < slideLowPosition){
                //let slide go up before moving to clear cone stack
            }
        }
    }

    public void bugOutAndPark() {
        bugOut = true;
        //put slide up a little
        slide_motor.setTargetPosition(slideLowPosition);
        while (slide_motor.getCurrentPosition() < slideLowPosition) {
            ServoOff();
            moveRobot(0, 0);
            //clear stack
        }
            if (START_LEFT) { //Drive to parking zone from cone stack in zone 1
                if (tagOfInterest.id == LEFT) {
                    //Stay here, just turn to 180+
                    driveStraight(DRIVE_SPEED, -5, 90);
                    turnToHeading(TURN_SPEED, 180);
                }
                else if (tagOfInterest.id == MIDDLE) {
                    // drive middle and turn
                    driveStraight(DRIVE_SPEED, -29, 90);
                    turnToHeading(TURN_SPEED, 180);
                }
                else {
                    driveStraight(DRIVE_SPEED, -58, 90);
                    turnToHeading(TURN_SPEED, 180);
                }
            }
            else {
                //Drive to parking zone from cone stack in zone 3
                if (tagOfInterest.id == LEFT) {
                    // drive to left
                    driveStraight(DRIVE_SPEED, -58, -90);
                    turnToHeading(TURN_SPEED, 180);
                }
                else if (tagOfInterest.id == MIDDLE) {
                    // drive to middle and turn
                    driveStraight(DRIVE_SPEED, -29, -90);
                    turnToHeading(TURN_SPEED, 180);
                }
                else {
                    driveStraight(DRIVE_SPEED, -5, -90);
                    turnToHeading(TURN_SPEED, 180);
                }
            }
            slide_motor.setPower(1);
            slide_motor.setTargetPosition(-20);
//            while (myOpMode.opModeIsActive() && bugOut) {
//                turnToHeading(TURN_SPEED, 180);
//                slide_motor.setPower(1);}
                slide_motor.setTargetPosition(-20);
            }


    /**
     *  Display the various control parameters while driving
     *
     * @param straight  Set to true if we are driving straight, and the encoder positions should be included in the telemetry.
     */
    private void sendTelemetry(boolean straight) {

        if (straight) {
            myOpMode.telemetry.addData("Motion", "Drive Straight");
            myOpMode.telemetry.addData("Target Pos L:R",  "%7d:%7d",      leftFrontTarget,  rightFrontTarget);
            myOpMode.telemetry.addData("Actual Pos L:R",  "%7d:%7d",      left_front.getCurrentPosition(),
                    right_front.getCurrentPosition());
        } else {
            myOpMode.telemetry.addData("Motion", "Turning");
        }

        myOpMode.telemetry.addData("Angle Target:Current", "%5.2f:%5.0f", targetHeading, robotHeading);
        myOpMode.telemetry.addData("Error:Steer",  "%5.1f:%5.1f", headingError, turnSpeed);
        myOpMode.telemetry.addData("Wheel Speeds L:R.", "%5.2f : %5.2f", leftSpeed, rightSpeed);
        myOpMode.telemetry.update();
    }

    /**
     * read the raw (un-offset Gyro heading) directly from the IMU
     */
    public double getRawHeading() {
        Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    /**
     * Reset the "offset" heading back to zero
     */
    public void resetHeading() {
        // Save a new heading offset equal to the current raw heading.
        headingOffset = getRawHeading();
        robotHeading = 0;
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        myOpMode.telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        myOpMode.telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        myOpMode.telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        myOpMode.telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        myOpMode.telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        myOpMode.telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        myOpMode.telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}
