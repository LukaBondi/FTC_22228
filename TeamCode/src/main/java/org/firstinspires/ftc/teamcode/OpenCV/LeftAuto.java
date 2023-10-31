package org.firstinspires.ftc.teamcode.OpenCV;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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

@Autonomous(name = "LeftAuto")
public class LeftAuto extends LinearOpMode {
    private IMU imu_IMU;
    private DcMotor M1;
    private DcMotor M2;
    private DcMotor M3;
    private DcMotor M4;
    private Servo Servo1;
    private DcMotor M5;
    private DcMotor M6;

    int ConeStack;
    int Low_junction;
    int Medium_junction;
    int High_junction;
    int parkTime;
    Orientation Angle;
    int LastError;
    double KP;
    double KD;
    float setPoint;
    ElapsedTime time2;
    float Error2;
    float D;

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagSize = 0.166;

    // Tag IDs of sleeve
    int Left = 7;
    int Middle = 8;
    int Right = 9;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagSize, fx, fy, cx, cy);
        Servo1 = hardwareMap.get(Servo.class, "Servo1");

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
        telemetry.setMsTransmissionInterval(50);

        imu_IMU = hardwareMap.get(IMU.class, "imu");
        M1 = hardwareMap.get(DcMotor.class, "M1");
        M2 = hardwareMap.get(DcMotor.class, "M2");
        M3 = hardwareMap.get(DcMotor.class, "M3");
        M4 = hardwareMap.get(DcMotor.class, "M4");
        Servo1 = hardwareMap.get(Servo.class, "Servo1");
        M5 = hardwareMap.get(DcMotor.class, "M5");
        M6 = hardwareMap.get(DcMotor.class, "M6");

        Robot_Setup();
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                boolean tagFound = false;

                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == Left || tag.id == Middle || tag.id == Right) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */
        if (tagOfInterest.id == Left) {
            telemetry.addLine("Tag found: Left");
            parkTime = 1;

        } else if (tagOfInterest == null || tagOfInterest.id == Middle) { //
            telemetry.addLine("Tag found: Middle");
            parkTime = 1;

        } else if (tagOfInterest.id == Right) {
            telemetry.addLine("Tag found: Right");
            parkTime = 1;
        }

        telemetry.addLine("Running main code... ");
        telemetry.update();
        forwardTime(0.3, 0.3);
        forwardTime(0.8, 1.5);
        turnRight(0.5, 33);
        junction_UP(High_junction);
        forwardTime(0.5, 0.5);
        sleep(100);
        Drop();
        backwardTime(0.5, 0.5);
        turnLeft(0.5, 127);
        junction_DOWN(High_junction - ConeStack);
        forwardTime(0.5, 1.8);
        PickUP();
        //backwardTime(0.5, 0.2);
        //sleep(300);
        junction_UP(Low_junction);
        backwardTime(0.5, 1.3);
        sleep(100);
        turnRight(0.5, 99);
        junction_UP(High_junction - Low_junction);
        forwardTime(0.5, 0.70);
        sleep(300);
        Drop();
        backwardTime(0.5, 0.3);
        junction_DOWN(High_junction);
        turnRight(0.5, 56);
        forwardTime(0.5, parkTime);

        /*You wouldn't have this in your autonomous, this is just to prevent the sample from ending*/
        while (opModeIsActive()) {
            sleep(20);
        }
    }

    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z * FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

    private void Robot_Setup() {
        ConeStack = 180; //  Lower the value, Lower the junction goes
        Low_junction = 1440;
        Medium_junction = 2440;
        High_junction = 3500;
        PickUP();
    }

    private void forwardTime(double speed, double Timer) {
        imu_IMU.resetYaw();
        M1.setDirection(DcMotorSimple.Direction.REVERSE);
        M2.setDirection(DcMotorSimple.Direction.REVERSE);
        M3.setDirection(DcMotorSimple.Direction.FORWARD);
        M4.setDirection(DcMotorSimple.Direction.FORWARD);
        LastError = 0;
        KP = 0.08;
        KD = 0.08;
        M1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        M2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        M3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        M4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setPoint = imu_IMU.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        time2 = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        if (opModeIsActive()) {
            time2.reset();
            while (time2.time() < Timer && opModeIsActive()) {
                Angle = imu_IMU.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                //telemetry.addData("Degrees", Error2.firstAngle);
                telemetry.update();
                Error2 = Angle.firstAngle - setPoint;
                D = Error2 - LastError;
                M1.setPower(speed + Error2 * KP + D * KD);
                M2.setPower(speed + Error2 * KP + D * KD);
                M3.setPower(speed - (Error2 * KP + D * KD));
                M4.setPower(speed - (Error2 * KP + D * KD));
                LastError = (int) Error2;
            }
            M1.setPower(0);
            M2.setPower(0);
            M3.setPower(0);
            M4.setPower(0);
        }
    }

    private void backwardTime(double speed, double Timer) {
        imu_IMU.resetYaw();
        M1.setDirection(DcMotorSimple.Direction.FORWARD);
        M2.setDirection(DcMotorSimple.Direction.FORWARD);
        M3.setDirection(DcMotorSimple.Direction.REVERSE);
        M4.setDirection(DcMotorSimple.Direction.REVERSE);
        LastError = 0;
        KP = 0.08;
        KD = 0.08;
        M1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        M2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        M3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        M4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setPoint = imu_IMU.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        time2 = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        if (opModeIsActive()) {
            time2.reset();
            while (time2.time() < Timer && opModeIsActive()) {
                Angle = imu_IMU.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                //telemetry.addData("Degrees", Error2.firstAngle);
                telemetry.update();
                Error2 = Angle.firstAngle - setPoint;
                D = Error2 - LastError;
                M1.setPower(speed - (Error2 * KP + D * KD));
                M2.setPower(speed - (Error2 * KP + D * KD));
                M3.setPower(speed + Error2 * KP + D * KD);
                M4.setPower(speed + Error2 * KP + D * KD);
                LastError = (int) Error2;
            }
            M1.setPower(0);
            M2.setPower(0);
            M3.setPower(0);
            M4.setPower(0);
            sleep(500);
        }
    }

    private void turnRight(double speed, int angles) {
        imu_IMU.resetYaw();
        M1.setDirection(DcMotorSimple.Direction.REVERSE);
        M2.setDirection(DcMotorSimple.Direction.REVERSE);
        M3.setDirection(DcMotorSimple.Direction.FORWARD);
        M4.setDirection(DcMotorSimple.Direction.FORWARD);
        M1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        M2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        M3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        M4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        M1.setPower(0.6);
        M2.setPower(0.6);
        M3.setPower(-0.6);
        M4.setPower(-0.6);
        sleep(5 * angles);
        while (opModeIsActive()) {
            Angle = imu_IMU.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            if (Angle.firstAngle < angles * -1 - 1.5) {
                M1.setPower(-speed);
                M2.setPower(-speed);
                M3.setPower(speed);
                M4.setPower(speed);
            } else if (Angle.firstAngle > angles * -1 + 1.5) {
                M1.setPower(speed);
                M2.setPower(speed);
                M3.setPower(-speed);
                M4.setPower(-speed);
            } else {
                M1.setPower(0);
                M2.setPower(0);
                M3.setPower(0);
                M4.setPower(0);
                break;
            }
        }
    }

    private void turnLeft(double speed, int angles){
        imu_IMU.resetYaw();
        M1.setDirection(DcMotorSimple.Direction.REVERSE);
        M2.setDirection(DcMotorSimple.Direction.REVERSE);
        M3.setDirection(DcMotorSimple.Direction.FORWARD);
        M4.setDirection(DcMotorSimple.Direction.FORWARD);
        M1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        M2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        M3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        M4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        M1.setPower(-0.6);
        M2.setPower(-0.6);
        M3.setPower(0.6);
        M4.setPower(0.6);
        sleep(5 * angles);
        while (opModeIsActive()) {
            Angle = imu_IMU.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            if (Angle.firstAngle < angles - 1.5) {
                M1.setPower(-speed);
                M2.setPower(-speed);
                M3.setPower(speed);
                M4.setPower(speed);
            } else if (Angle.firstAngle > angles + 1.5) {
                M1.setPower(speed);
                M2.setPower(speed);
                M3.setPower(-speed);
                M4.setPower(-speed);
            } else {
                M1.setPower(0);
                M2.setPower(0);
                M3.setPower(0);
                M4.setPower(0);
                break;
            }
        }
    }

    private void Mode1() {
        forwardTime(0.8, 0.5);
    }

    private void Mode2() {
        backwardTime(0.5, 1.2);
    }

    private void Drop() {
        Servo1.setPosition(-0.2);
    }

    private void PickUP() {
        Servo1.setPosition(0.55);
        sleep(500);
    }

    private void printIMU() {
        imu_IMU.resetYaw();
        while (opModeIsActive()) {
            Angle = imu_IMU.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("Degrees", Angle.firstAngle);
            telemetry.update();
        }
    }

    private void junction_DOWN(int junction) {
        M5.setDirection(DcMotorSimple.Direction.REVERSE);
        M6.setDirection(DcMotorSimple.Direction.FORWARD);
        M5.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        M6.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        M5.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        M6.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        M5.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        M6.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        M5.setTargetPosition(junction);
        M6.setTargetPosition(junction);
        M5.setPower(1);
        M6.setPower(1);
        while (!(!M5.isBusy() && !M6.isBusy() && opModeIsActive())) {
        }
        M5.setPower(0);
        M6.setPower(0);
        sleep(500);
    }

    private void junction_UP(int junction) {
        M5.setDirection(DcMotorSimple.Direction.FORWARD);
        M6.setDirection(DcMotorSimple.Direction.REVERSE);
        M5.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        M6.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        M5.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        M6.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        M5.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        M6.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        M5.setTargetPosition(junction);
        M6.setTargetPosition(junction);
        M5.setPower(1);
        M6.setPower(1);
        while (!(!M5.isBusy() && !M6.isBusy() && opModeIsActive())) {
        }
        M5.setPower(0);
        M6.setPower(0);
    }
}
