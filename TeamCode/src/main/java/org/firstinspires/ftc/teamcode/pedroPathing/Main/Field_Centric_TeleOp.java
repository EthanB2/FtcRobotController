package org.firstinspires.ftc.teamcode.pedroPathing.Main;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp(name="Main Tele-Op Field Centric", group="Linear OpMode")
//Disabled
public class Field_Centric_TeleOp extends LinearOpMode {


        // Declare OpMode members for each of the 4 motors.
        private ElapsedTime runtime = new ElapsedTime();
        private DcMotor leftFrontDrive = null;
        private DcMotor leftBackDrive = null;
        private DcMotor rightFrontDrive = null;
        private DcMotor rightBackDrive = null;
        private DcMotor tiltMotor = null;
        private DcMotor extendMotor = null;
        private DcMotor liftMotor = null;
//        private Servo LClaw = null;
//        private Servo RClaw = null;
//        private CRServo intake = null;
        private Servo specimenClaw = null;
     //   private DcMotor lifter = null;

        private IMU imu;

        //speed variables
        private double motorSpeed = 1;
        private double tiltSpeed = 0.75;
        private double extendSpeed = 0.75;
        private double gravity = 0;

    public enum ArmState {
        Start,
        Move
    }
    ArmState Armstate = ArmState.Start;
        //Declaring Servo variables
    /*CRServo claw1 = null;
    CRServo claw2 = null;*/

        @Override
        public void runOpMode() {

            //Initializing hardware motor variables.
            leftFrontDrive = hardwareMap.get(DcMotor.class, "frontLeft");
            leftBackDrive = hardwareMap.get(DcMotor.class, "backLeft");
            rightFrontDrive = hardwareMap.get(DcMotor.class, "frontRight");
            rightBackDrive = hardwareMap.get(DcMotor.class, "backRight");
            imu = hardwareMap.get(IMU.class, "imu");

            //Servo hardware map init
        //    LClaw = hardwareMap.get(Servo.class, "Left_Claw");
        //    RClaw = hardwareMap.get(Servo.class, "Right_Claw");
        //    intake = hardwareMap.get(CRServo.class, "Intake");
        //    specimenClaw = hardwareMap.get(Servo.class, "Specimen_Claw");

            //Initializing extra motors
            tiltMotor = hardwareMap.get(DcMotor.class, "tilt");
            extendMotor = hardwareMap.get(DcMotor.class, "extend");
            liftMotor = hardwareMap.get(DcMotor.class, "lift");
          //  lifter = hardwareMap.get(DcMotor.class, "lifter");

            //Setting Servo motors variables.
            //claw1 = hardwareMap.get(CRServo.class, "claw1");
            //claw2 = hardwareMap.get(CRServo.class, "claw2");

            //setting DcMotor variables to specify primary direction.
            leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
            leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
            rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
            rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

            //servo set direction
 //           intake.setDirection(DcMotor.Direction.REVERSE);

            //setting DcMotor variables for extra motors
            tiltMotor.setDirection(DcMotor.Direction.FORWARD);
            extendMotor.setDirection(DcMotor.Direction.REVERSE);

            tiltMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            tiltMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            extendMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            extendMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         //   lifter.setDirection(DcMotor.Direction.FORWARD);

            //lift motor,  declaring forward operation, and usage of encoder
            liftMotor.setDirection(DcMotor.Direction.FORWARD);
            liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            //Zero power behavior for Motors
            leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            tiltMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            extendMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            //set initial position/power
            leftFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightFrontDrive.setPower(0);
            rightBackDrive.setPower(0);
//            tiltMotor.setPower(0);
//            extendMotor.setPower(0);
//            RClaw.setPosition(.75);
//            LClaw.setPosition(.75);

            RevHubOrientationOnRobot orientationOnRobot =
                    new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                            RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD);
            imu.initialize(new IMU.Parameters(orientationOnRobot));
            imu.resetYaw();



            // Waiting for game to start.
            telemetry.addData("Status", "Initialized");
            telemetry.update();

            //Reset runtime while still waiting for game to start.
            waitForStart();
            runtime.reset();

            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {
                double max;

                // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
                double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
                double lateral = gamepad1.left_stick_x;
                double yaw = gamepad1.right_stick_x;

                if (gamepad1.left_bumper) {
                    motorSpeed = 0.4;
                } else {
                    motorSpeed = 0.75;
                }

                if (gamepad2.left_bumper) {
                    extendSpeed = 0.5;
                    tiltSpeed = 0.4;
                } else {
                    extendSpeed = 1;
                    tiltSpeed = 0.75;
                }

                if (gamepad1.y) {
                    imu.resetYaw();
                }

                double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

                double rotX = axial * Math.cos(botHeading) - lateral * Math.sin(botHeading);
                double rotY = axial * Math.sin(botHeading) + lateral * Math.cos(botHeading);

                // Combine the joystick requests for each axis-motion to determine each wheel's power.
                // Set up a variable for each drive wheel to save the power level for telemetry.

                double leftFrontPower = motorSpeed*(rotX + rotY + yaw);
                double rightFrontPower = motorSpeed*(rotX - rotY - yaw);
                double leftBackPower = motorSpeed*(rotX - rotY + yaw);
                double rightBackPower = motorSpeed*(rotX + rotY - yaw);

                double tiltPower = tiltSpeed*(gamepad2.left_stick_y);
                double extendPower = extendSpeed*(-gamepad2.right_stick_y);

                // Normalize the values so no wheel power exceeds 100%
                // This ensures that the robot maintains the desired motion.
                max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
                max = Math.max(max, Math.abs(leftBackPower));
                max = Math.max(max, Math.abs(rightBackPower));

                if (max > 1.0) {
                    leftFrontPower /= max;
                    rightFrontPower /= max;
                    leftBackPower /= max;
                    rightBackPower /= max;
                }

                // This is test code:
                //
                // Uncomment the following code to test your motor directions.
                // Each button should make the corresponding motor run FORWARD.
                //   1) First get all the motors to take to correct positions on the robot
                //      by adjusting your Robot Configuration if necessary.
                //   2) Then make sure they run in the correct direction by modifying the
                //      the setDirection() calls above.
                // Once the correct motors move in the correct direction re-comment this code.

            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

                // Send calculated power to wheels
                leftFrontDrive.setPower(leftFrontPower);
                rightFrontDrive.setPower(rightFrontPower);
                leftBackDrive.setPower(leftBackPower);
                rightBackDrive.setPower(rightBackPower);

                if (tiltMotor.getCurrentPosition() >= 1300) {
                    gravity = 0.01;
                } else {
                    gravity = 0;
                }

                //temporary override code.  Need to change encoder values to actual stop points.
                if(gamepad2.right_bumper && Armstate != ArmState.Move){
                    extendMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                extendMotor.setPower(extendPower);
                } else if (extendMotor.getCurrentPosition() >= 1500 && tiltMotor.getCurrentPosition()<1500 && gamepad2.right_stick_y<-0.1 && Armstate != ArmState.Move) {
                    extendMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    extendMotor.setPower(0);
                } else if (Armstate != ArmState.Move){
                    extendMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    extendMotor.setPower(extendPower+gravity);
                }

                if(gamepad2.right_bumper && Armstate != ArmState.Move){
                    tiltMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    tiltMotor.setPower(tiltPower);
                } else if (extendMotor.getCurrentPosition() >= 1500 && tiltMotor.getCurrentPosition()<1500 && gamepad2.left_stick_y<-0.1 && Armstate != ArmState.Move) {
                    tiltMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    tiltMotor.setPower(0);
                }else if (Armstate != ArmState.Move && tiltMotor.getCurrentPosition()>=2828 && gamepad2.left_stick_y>0.1) {
                    tiltMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    tiltMotor.setPower(0);
                }
                else if (Armstate != ArmState.Move) {
                    tiltMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    tiltMotor.setPower(tiltPower);
                }
  //              original code here, remove all temp override code above if revert
//                if (extendMotor.getCurrentPosition() < -1000){
//                    extendMotor.setPower(extendPower);
//                } else {
//                    extendMotor.setPower(-1);
//                }
//
//                if (extendMotor.getCurrentPosition() < -1000){
//                    extendMotor.setPower(-1.0);
//                }

//code for Lifter
//                if (gamepad1.left_trigger>0.1 && gamepad1.right_trigger<=0.1) {
//                    lifter.setPower(1);
//                } else if (gamepad1.right_trigger>0.1 && gamepad1.left_trigger<=0.1) {
//                    lifter.setPower(-1);
//                } else {
//                    lifter.setPower(0);
//                }


//code for claw intake
//                if (gamepad2.left_bumper) {
//                    //open
//                    RClaw.setPosition(0.5);
//                    LClaw.setPosition(1);
//                } else if (gamepad2.right_bumper) {
//                    //closed
//                    RClaw.setPosition(0.75);
//                    LClaw.setPosition(0.75);
//                }

//code for active intake
//                if (gamepad2.right_trigger>0.1){
//                    intake.setPower(1);
//                } else if (gamepad2.left_trigger>0.1) {
//                    intake.setPower(-1);
//                } else {
//                    intake.setPower(0);
//                }

//code for specimen claw
//                if (gamepad2.b) {
//                    specimenClaw.setPosition(0.5);
//                } else {
//                    specimenClaw.setPosition(-0.1);
//                }

                switch (Armstate) {
                    case Start:
                        if (gamepad2.a){
                            //ground
                            tiltMotor.setTargetPosition(0);
                            extendMotor.setTargetPosition(0);
                            tiltMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            extendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            tiltMotor.setPower(0.75);
                            extendMotor.setPower(0.75);
                            tiltMotor.setTargetPosition(60);
                            extendMotor.setTargetPosition(2);
                            Armstate = ArmState.Move;
                        }

                        if (gamepad2.y){
                            //High Basket
                            tiltMotor.setTargetPosition(0);
                            extendMotor.setTargetPosition(0);
                            tiltMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            extendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            tiltMotor.setPower(0.75);
                            extendMotor.setPower(0.75);
                            tiltMotor.setTargetPosition(2099);
                            extendMotor.setTargetPosition(2521);
                            Armstate = ArmState.Move;
                        }

                        if (gamepad2.dpad_down) {
                            //Specimen Pick Up
                            tiltMotor.setTargetPosition(0);
                            extendMotor.setTargetPosition(0);
                            tiltMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            extendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            tiltMotor.setPower(0.75);
                            extendMotor.setPower(0.75);
                            tiltMotor.setTargetPosition(276);
                            extendMotor.setTargetPosition(2);
                            Armstate = ArmState.Move;
                        }
                        if (gamepad2.dpad_up){
                            //Specimen Place
                            tiltMotor.setTargetPosition(0);
                            extendMotor.setTargetPosition(0);
                            tiltMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            extendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            tiltMotor.setPower(0.75);
                            extendMotor.setPower(0.75);
                            tiltMotor.setTargetPosition(1778);
                            extendMotor.setTargetPosition(1400);
                            Armstate = ArmState.Move;
                        }
                    break;
                    case Move:
                        if (Math.abs(tiltMotor.getCurrentPosition() - tiltMotor.getTargetPosition()) < 10 && Math.abs(extendMotor.getCurrentPosition() - extendMotor.getTargetPosition()) < 10) {
                            tiltMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            extendMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            Armstate = ArmState.Start;
                        }
                        break;
                    default:
                        // should never be reached, as liftState should never be null
                        Armstate = ArmState.Start;
                }
                if (gamepad2.right_bumper && Armstate != ArmState.Start) {
                    Armstate = ArmState.Start;
                }

                if (gamepad1.right_trigger >= 0.1 && !(gamepad1.left_trigger >= 0.1)){
                    liftMotor.setPower(1);
                } else if(gamepad1.left_trigger >= 0.1 && !(gamepad1.right_trigger >= 0.1)) {
                    liftMotor.setPower(-1);
                } else {
                    liftMotor.setPower(0);
                }




                // Show the elapsed game time and wheel power.
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
                telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
                telemetry.addData("Front Left", leftFrontDrive.getCurrentPosition());
                telemetry.addData("Front Right", rightFrontDrive.getCurrentPosition());
                telemetry.addData("Back Left", leftBackDrive.getCurrentPosition());
                telemetry.addData("Back Right", rightBackDrive.getCurrentPosition());
                telemetry.addData("tilt Power", "%4.2f", tiltMotor.getPower());
                telemetry.addData("extension Power", "%4.2f", extendMotor.getPower());
                telemetry.addData("lifter Power", "%4.2f", liftMotor.getPower());
                telemetry.addData("tilt Pos", tiltMotor.getCurrentPosition());
                telemetry.addData("extension Pos", extendMotor.getCurrentPosition());
                telemetry.addData("State", Armstate);
                telemetry.addData("Robot Angle", imu.getRobotYawPitchRollAngles());
                telemetry.update();

                //Determine claw power based on gamepad "bumper" input
            /*if(gamepad2.left_bumper ){
                claw1.setPower(1);
                claw2.setPower(-1);
            }
            else if (gamepad2.right_bumper){
                claw1.setPower(-1);
                claw2.setPower(1);
            }
            else{
                claw1.setPower(0);
                claw2.setPower(0);
            }*/


            }

        }
    }


