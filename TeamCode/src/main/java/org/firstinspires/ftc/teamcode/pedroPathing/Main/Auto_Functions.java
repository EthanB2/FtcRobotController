/* Created by Phil Malone. 2023.
    This class illustrates my simplified Odometry Strategy.
    It implements basic straight line motions but with heading and drift controls to limit drift.
    See the readme for a link to a video tutorial explaining the operation and limitations of the code.
 */

package org.firstinspires.ftc.teamcode.pedroPathing.Main;

//import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

public class Auto_Functions {

    Boolean exit = false;
    // ---  Private Members

    // Hardware interface Objects

    private DcMotor tiltMotor;
    private DcMotor extendMotor;

    private Servo Claw = null;
//    private CRServo intake = null;

    private LinearOpMode myOpMode;
    private ElapsedTime holdTimer = new ElapsedTime();  // User for any motion requiring a hold time or timeout.

    private boolean showTelemetry     = false;

    private double tiltValue;
    private double extendValue;



    // Robot Constructor
    public Auto_Functions(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public Auto_Functions(Test_Auto testAuto) {
    }

    /**
     * Robot Initialization:
     *  Use the hardware map to Connect to devices.
     *  Perform any set-up all the hardware devices.
     * @param showTelemetry  Set to true if you want telemetry to be displayed by the robot sensor/drive functions.
     */
    public void initialize(boolean showTelemetry)
    {
        // Initialize the hardware variables. Note that the strings used to 'get' each
        // motor/device must match the names assigned during the robot configuration.
        tiltMotor = myOpMode.hardwareMap.get(DcMotor.class, "tilt");
        extendMotor = myOpMode.hardwareMap.get(DcMotor.class, "extend");

        Claw = myOpMode.hardwareMap.get(Servo.class, "Claw");

        // !!!  Set the drive direction.
        tiltMotor.setDirection(DcMotor.Direction.FORWARD);
        extendMotor.setDirection(DcMotor.Direction.REVERSE);



        // Set all hubs to use the AUTO Bulk Caching mode for faster encoder reads
        List<LynxModule> allHubs = myOpMode.hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }


    }

    //  ########################  Mid level control functions.  #############################3#



    //  ########################  Low level control functions.  ###############################

    public void tiltArm(double angle, double power) {
        //32.22 ticks/degree
        tiltValue=angle*32.22;
        while (myOpMode.opModeIsActive()) {
            tiltMotor.setTargetPosition((int) tiltValue);
            tiltMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            tiltMotor.setPower(power);
            if (Math.abs(tiltMotor.getCurrentPosition()-tiltValue)<5) {
                break;   // Exit loop if we are in position.
            }
        }


    }

    public void extendArm(double inches, double power) {
        //0 ticks = 0 in, 1276 ticks = 15.5625
        //81.99 ticks/inch
        extendValue=inches*81.99;
        while (myOpMode.opModeIsActive()) {
            extendMotor.setTargetPosition((int) extendValue);
            extendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            extendMotor.setPower(power);
            if (Math.abs(extendMotor.getCurrentPosition()-extendValue)<5) {
                break;   // Exit loop if we are in position.
            }
        }
    }

    public void closeClaw() {Claw.setPosition(0);}

    public void openClaw() {Claw.setPosition(0.3);}

    public void closeClawLoose() {Claw.setPosition(0.06);}
}
