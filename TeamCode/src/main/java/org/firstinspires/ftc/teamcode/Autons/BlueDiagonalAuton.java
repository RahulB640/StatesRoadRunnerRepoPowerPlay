package org.firstinspires.ftc.teamcode.Autons;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.CameraVision.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.customUtil.Constants;
import org.firstinspires.ftc.teamcode.customUtil.RobotHardwareMap;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.ArrayList;

@Config
@Autonomous(name="Blue Diagonal Auton", group="Auton")
public class BlueDiagonalAuton extends LinearOpMode {

    RobotHardwareMap robot = new RobotHardwareMap();

    public static double X = 64.75;
    public static double Y = -2.2;
    public static double ANGLE = 0;
    public static double time = 1;


    //Time for the lasy susan to spin from either side before reaching the front and letting slides drop
    public static double intakeDelay = 0.4;

    //The amount of time to wait for slides to raise before starting the susan
    public static double scoreDelay = 0.9;

    //Amount of time to wait for the slides to drop before grabbing
    public static double grabDelay= 1.65;

    //Amount of time for the servo to release
    public static double scoreWait = 0.2;

    //Time needed to wait for slides to drop
    public static double dropTime = 0.6;

    double waitTime = 0.1;

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    public static double fx = 250;
    public static double fy = 250;
    public static double cx = 250;
    public static double cy = 250;
    // UNITS ARE METERS
    public static double tagsize = 0.166;

    //Tag ID 1, 2, 3 from the 36h11 family
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    public static int height = 1280;
    public static int width = 720;

    AprilTagDetection tagOfInterest = null;

    int zone = 2;

    @Override
    public void runOpMode() {
        robot.initialize(hardwareMap);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

//        robot.ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_OCEAN_PALETTE);


        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        TrajectorySequence scorePreloaded = drive.trajectorySequenceBuilder(startPose)
                .splineToLinearHeading(new Pose2d(2, -2.5, Math.toRadians(0)), 0)
                .addTemporalMarker(0, () -> {robot.slideMotor.setTargetPosition(Constants.highJunctionSlideTicks);})
                .addTemporalMarker(scoreDelay, () -> {robot.lazySusanSpinner.setTargetPosition(Constants.lasySusanLeft);})
                .splineToLinearHeading(new Pose2d(61.5, -2.4, Math.toRadians(0)), 0)
                .build();

        TrajectorySequence intakeCone1 = drive.trajectorySequenceBuilder(scorePreloaded.end())
                .waitSeconds(0.1)
                .addTemporalMarker(0, () -> {robot.lazySusanSpinner.setTargetPosition(Constants.lasySusanForward);})
                .addTemporalMarker(intakeDelay, () -> {robot.slideMotor.setTargetPosition(Constants.coneHeight5);})
//                .addTemporalMarker(grabDelay + intakeDelay, () -> {robot.clawServo.setPosition(Constants.clawServoClosedPosition);})
                .splineToLinearHeading(new Pose2d(51, -3, Math.toRadians(-90)), Math.toRadians(-90))
                .splineToLinearHeading(new Pose2d(51, -28, Math.toRadians(-90)), Math.toRadians(-90))
                .build();


        TrajectorySequence scoreCone1 = drive.trajectorySequenceBuilder(intakeCone1.end())
                .waitSeconds(0.2)
                .addTemporalMarker(0, () -> {robot.slideMotor.setTargetPosition(Constants.highJunctionSlideTicks);})
                .addTemporalMarker(scoreDelay, () -> {robot.lazySusanSpinner.setTargetPosition(Constants.lasySusanLeft);})
                .lineToLinearHeading(new Pose2d(52.6, 8, Math.toRadians(-90)))
                .waitSeconds(scoreWait)
                .build();

        TrajectorySequence intakeCone2 = drive.trajectorySequenceBuilder(scoreCone1.end())
                .waitSeconds(0.1)
                .addTemporalMarker(0, () -> {robot.lazySusanSpinner.setTargetPosition(Constants.lasySusanForward);})
                .addTemporalMarker(intakeDelay, () -> {robot.slideMotor.setTargetPosition(Constants.coneHeight4);})
//                .addTemporalMarker(grabDelay + intakeDelay, () -> {robot.clawServo.setPosition(Constants.clawServoClosedPosition);})
                .lineToLinearHeading(new Pose2d(51, -28, Math.toRadians(-90)))
                .waitSeconds(dropTime)
                .build();

        TrajectorySequence scoreCone2 = drive.trajectorySequenceBuilder(intakeCone2.end())
                .addTemporalMarker(0, () -> {robot.slideMotor.setTargetPosition(Constants.highJunctionSlideTicks);})
                .addTemporalMarker(scoreDelay, () -> {robot.lazySusanSpinner.setTargetPosition(Constants.lasySusanLeft);})
                .lineToLinearHeading(new Pose2d(52.6, 8, Math.toRadians(-90)))
                .waitSeconds(scoreWait)
                .build();

        TrajectorySequence intakeCone3 = drive.trajectorySequenceBuilder(scoreCone2.end())
                .waitSeconds(0.1)
                .addTemporalMarker(0, () -> {robot.lazySusanSpinner.setTargetPosition(Constants.lasySusanForward);})
                .addTemporalMarker(intakeDelay, () -> {robot.slideMotor.setTargetPosition(Constants.coneHeight3);})
//                .addTemporalMarker(grabDelay + intakeDelay, () -> {robot.clawServo.setPosition(Constants.clawServoClosedPosition);})
                .lineToLinearHeading(new Pose2d(51, -28, Math.toRadians(-90)))
                .waitSeconds(dropTime)
                .build();

        TrajectorySequence scoreCone3 = drive.trajectorySequenceBuilder(intakeCone3.end())
                .addTemporalMarker(0, () -> {robot.slideMotor.setTargetPosition(Constants.highJunctionSlideTicks);})
                .addTemporalMarker(scoreDelay, () -> {robot.lazySusanSpinner.setTargetPosition(Constants.lasySusanLeft);})
                .lineToLinearHeading(new Pose2d(52.6, 8, Math.toRadians(-90)))
                .waitSeconds(scoreWait)
                .build();

        TrajectorySequence intakeCone4 = drive.trajectorySequenceBuilder(scoreCone3.end())
                .waitSeconds(0.1)
                .addTemporalMarker(0, () -> {robot.lazySusanSpinner.setTargetPosition(Constants.lasySusanForward);})
                .addTemporalMarker(intakeDelay, () -> {robot.slideMotor.setTargetPosition(Constants.coneHeight2);})
//                .addTemporalMarker(grabDelay + intakeDelay, () -> {robot.clawServo.setPosition(Constants.clawServoClosedPosition);})
                .lineToLinearHeading(new Pose2d(51, -28, Math.toRadians(-90)))
                .waitSeconds(dropTime)
                .build();

        TrajectorySequence scoreCone4 = drive.trajectorySequenceBuilder(intakeCone4.end())
                .addTemporalMarker(0, () -> {robot.slideMotor.setTargetPosition(Constants.highJunctionSlideTicks);})
                .addTemporalMarker(scoreDelay, () -> {robot.lazySusanSpinner.setTargetPosition(Constants.lasySusanLeft);})
                .lineToLinearHeading(new Pose2d(52.6, 8, Math.toRadians(-90)))
                .waitSeconds(scoreWait)
                .build();

        TrajectorySequence goToZone1 = drive.trajectorySequenceBuilder(scoreCone4.end())
                .waitSeconds(0.1)
                .addTemporalMarker(0, () -> {robot.lazySusanSpinner.setTargetPosition(Constants.lasySusanForward);})
                .addTemporalMarker(intakeDelay, () -> {robot.slideMotor.setTargetPosition(Constants.coneHeight2);})
                .addTemporalMarker(grabDelay + intakeDelay, () -> {robot.clawServo.setPosition(Constants.clawServoClosedPosition);})
                .lineToLinearHeading(new Pose2d(51.5, 23, Math.toRadians(-90)))
                .build();

        TrajectorySequence goToZone2 = drive.trajectorySequenceBuilder(scoreCone4.end())
                .waitSeconds(0.1)
                .addTemporalMarker(0, () -> {robot.lazySusanSpinner.setTargetPosition(Constants.lasySusanForward);})
                .addTemporalMarker(intakeDelay, () -> {robot.slideMotor.setTargetPosition(Constants.coneHeight2);})
                .lineToLinearHeading(new Pose2d(51.5, -2.5, Math.toRadians(-90)))
                .build();

        TrajectorySequence goToZone3 = drive.trajectorySequenceBuilder(scoreCone4.end())
                .waitSeconds(0.1)
                .addTemporalMarker(0, () -> {robot.lazySusanSpinner.setTargetPosition(Constants.lasySusanForward);})
                .addTemporalMarker(intakeDelay, () -> {robot.slideMotor.setTargetPosition(Constants.coneHeight2);})
                .addTemporalMarker(grabDelay + intakeDelay, () -> {robot.clawServo.setPosition(Constants.clawServoClosedPosition);})
                .lineToLinearHeading(new Pose2d(51.5, -28.5, Math.toRadians(-90)))
                .build();





        //Camera Vision
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        camera = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
//        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
//
//        camera.setPipeline(aprilTagDetectionPipeline);
//        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
//        {
//            @Override
//            public void onOpened()
//            {
//                camera.startStreaming(height,width, OpenCvCameraRotation.UPRIGHT);
//            }
//
//            @Override
//            public void onError(int errorCode)
//            {
//
//            }
//        });
//
//        telemetry.setMsTransmissionInterval(50);
//
//
//
//
//        robot.clawServo.setPosition(Constants.clawServoClosedPosition);
        waitForStart();

//        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
//        for(AprilTagDetection tag : currentDetections)
//        {
//            if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
//            {
//                tagOfInterest = tag;
//                break;
//            }
//        }
//
//        if (tagOfInterest == null || tagOfInterest.id == MIDDLE){
//            zone = 2;
//        }
//        else if (tagOfInterest.id == LEFT){
//            zone = 1;
//        }
//        else if (tagOfInterest.id == RIGHT){
//            zone = 3;
//        }
//
//        telemetry.addData("Zone", zone);
//        telemetry.update();
//        camera.stopStreaming();
//        camera.stopRecordingPipeline();

//        double beginTime = getRuntime();

        drive.followTrajectorySequence(scorePreloaded);
//        robot.clawServo.setPosition(Constants.clawServoOpenPosition);

        drive.followTrajectorySequence(intakeCone1);
//        robot.clawServo.setPosition(Constants.clawServoClosedPosition);

        drive.followTrajectorySequence(scoreCone1);
//        robot.clawServo.setPosition(Constants.clawServoOpenPosition);

        drive.followTrajectorySequence(intakeCone2);
//        robot.clawServo.setPosition(Constants.clawServoClosedPosition);

        drive.followTrajectorySequence(scoreCone2);
//        robot.clawServo.setPosition(Constants.clawServoOpenPosition);

        drive.followTrajectorySequence(intakeCone3);
//        robot.clawServo.setPosition(Constants.clawServoClosedPosition);

        drive.followTrajectorySequence(scoreCone3);
//        robot.clawServo.setPosition(Constants.clawServoOpenPosition);

        drive.followTrajectorySequence(intakeCone4);
//        robot.clawServo.setPosition(Constants.clawServoClosedPosition);

        drive.followTrajectorySequence(scoreCone4);
//        robot.clawServo.setPosition(Constants.clawServoOpenPosition);

//        telemetry.addData("Cone 4:", (getRuntime() - beginTime));
//
//        if (zone == 1){
//            drive.followTrajectorySequence(goToZone1);
//        }
//        else if (zone == 2){
//            drive.followTrajectorySequence(goToZone2);
//        }
//        else if (zone == 3){
//            drive.followTrajectorySequence(goToZone3);
//        }
        sleep(10000);

//        telemetry.addData("Runtime:", (getRuntime() - beginTime));
//        telemetry.update();


    }




    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
//        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
//        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
//        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
//        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
//        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
//        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}






