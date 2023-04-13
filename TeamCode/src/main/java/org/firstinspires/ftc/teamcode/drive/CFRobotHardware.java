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

    //tracking wheels
    private DcMotor left_tracking = null;
    private DcMotor right_tracking = null;
    private DcMotor front_tracking = null;

    private BNO055IMU imu       = null;  // Control/Expansion Hub IMU

    private double robotHeading  = 0;
    private double headingOffset = 0;
    private double headingError  = 0;


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

        left_front.setDirection(DcMotor.Direction.FORWARD);
        left_rear.setDirection(DcMotor.Direction.FORWARD);
        right_front.setDirection(DcMotor.Direction.REVERSE);
        right_rear.setDirection(DcMotor.Direction.REVERSE);

        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_rear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_rear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_rear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_rear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


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

    // **********  LOW Level driving functions.  ********************


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
