package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.customUtil.Constants;
import org.firstinspires.ftc.teamcode.customUtil.RobotHardwareMap;


@TeleOp(name = "Robot Oriented TeleOp", group = "TeleOps")
public class robotOrientedTeleOp extends LinearOpMode {

    RobotHardwareMap robot = new RobotHardwareMap();


    @Override
    public void runOpMode() {
        robot.initialize(hardwareMap);



        // initializes constants
        double drive;
        double strafe;
        double turn;

        double frontLeftPower;
        double frontRightPower;
        double backLeftPower;
        double backRightPower;

        double maxMotorSpeed = 0.7;

        int heightDifferential = 0;

        double maxPower;




        waitForStart();

        // Driver code
        double beginTime = getRuntime();
        while (opModeIsActive()) {

            drive = -gamepad1.left_stick_y;
            strafe = gamepad1.left_stick_x;
            turn = gamepad1.right_stick_x;


            frontLeftPower = drive + strafe + turn;
            frontRightPower = drive - strafe - turn;
            backLeftPower = drive - strafe + turn;
            backRightPower = drive + strafe - turn;

            // caps max power to ensure motor is not used at 100% capacity, which would damage it.
            if (Math.abs(frontLeftPower) > 1 || Math.abs(frontRightPower) > 1 || Math.abs(backLeftPower) > 1 || Math.abs(backRightPower) > 1){

                maxPower = Math.max(Math.abs(frontLeftPower), Math.max(Math.abs(frontRightPower), Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));

                //fix problem
                frontLeftPower /= maxPower;
                frontRightPower /= maxPower;
                backLeftPower /= maxPower;
                backRightPower /= maxPower;
            }

            // sets motor powers
            robot.frontLeftMotor.setPower(frontLeftPower * maxMotorSpeed);
            robot.frontRightMotor.setPower(frontRightPower * maxMotorSpeed);
            robot.backRightMotor.setPower(backRightPower * maxMotorSpeed);
            robot.backLeftMotor.setPower(backLeftPower * maxMotorSpeed);

            //For Slides besides ground level
            if(gamepad2.dpad_left){
                robot.slideMotor.setTargetPosition(Constants.lowJunctionSlideTicks);
            }
            else if (gamepad2.dpad_right){
                robot.slideMotor.setTargetPosition(Constants.middleJunctionSlideTicks);
            }
            else if (gamepad2.dpad_up){
                robot.slideMotor.setTargetPosition(Constants.highJunctionSlideTicks);
            }

            //If the slides are in the front
            if (Math.abs(robot.lazySusanSpinner.getCurrentPosition()) < 10){
                if(gamepad2.dpad_down){
                    //robot.clawServo.setPosition(Constants.clawServoClosedPosition);
                    robot.slideMotor.setTargetPosition(Constants.slideGroundLevelTicks);
                }
                if (gamepad2.y){
                    //5 Cone Stack
                    robot.slideMotor.setTargetPosition(Constants.coneHeight5);
                }
                else if (gamepad2.x){
                    //4 Cone Stack
                    robot.slideMotor.setTargetPosition(Constants.coneHeight4);
                }
                else if (gamepad2.b){
                    //3 Stack Cone
                    robot.slideMotor.setTargetPosition(Constants.coneHeight3);
                }
                else if (gamepad2.a){
                    //2 Stack Cone
                    robot.slideMotor.setTargetPosition(Constants.coneHeight2);
                }
            }

            if (gamepad1.dpad_up){
                robot.lazySusanSpinner.setTargetPosition(Constants.lasySusanForward);
            }

            if (robot.slideMotor.getCurrentPosition() > (0.8*Constants.lowJunctionSlideTicks)) {
                if (gamepad1.dpad_left) {
                    robot.lazySusanSpinner.setTargetPosition(Constants.lasySusanLeft);
                } else if (gamepad1.dpad_right) {
                    robot.lazySusanSpinner.setTargetPosition(Constants.lasySusanRight);
                }
            }

            if (gamepad2.left_bumper){
                robot.clawServo.setPosition(Constants.clawServoClosedPosition);
            }
            else if (gamepad2.right_bumper){
                robot.clawServo.setPosition(Constants.clawServoOpenPosition);
            }









            //slides
            //cone stack heights
            //lasy susan
            //servo
            //leds
            //height differential
            //scope in cod thing











            //TODO: Telemetry
//            telemetry.addData("slideMotor Ticks ", robot.slideMotor.getCurrentPosition());
//            telemetry.addData("Slide Power", robot.slideMotor.getPower());
//            telemetry.addData("Height Differential", heightDifferential);
//            telemetry.addData("Slide Height", robot.slideHeightSensor.getDistance(DistanceUnit.MM));
//            telemetry.addData(" ", " ");
//
//            telemetry.addData("Cone Sensor Distance", robot.coneSensorDistance.getDistance(DistanceUnit.MM));
//            telemetry.addData("Cone Sensor Red", robot.coneSensorColor.red());
//            telemetry.addData("Cone Sensor Green", robot.coneSensorColor.green());
//            telemetry.addData("Cone Sensor Blue", robot.coneSensorColor.blue());
//
//            telemetry.addData("Lazy Susan Position", robot.lazySusanSpinner.getCurrentPosition());
//            telemetry.addData("Lazy Susan Target Position", robot.lazySusanSpinner.getTargetPosition());
//            telemetry.addData("Lazy Susan Power", robot.lazySusanSpinner.getPower());

//
//            telemetry.addData("Runtime", getRuntime() - beginTime);

            telemetry.update();
        }
    }
}













