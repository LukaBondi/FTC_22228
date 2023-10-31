package org.firstinspires.ftc.teamcode.OpenCV;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "ENCODERByEasyKidsRobotics")
public class EncoderByEasyKidsRobotics extends LinearOpMode {

    private IMU imu_IMU;
    private DcMotor M1;
    private DcMotor M2;
    private DcMotor M3;
    private DcMotor M4;
    private BNO055IMU imu;
    private Servo Servo1;
    private DcMotor M5AsDcMotor;
    private DcMotor M6AsDcMotor;

    int ConeStack;
    int Low_junction;
    int High_junction;
    Orientation Angle;
    int LastError;
    double KP;
    double KD;
    float setPoint;
    ElapsedTime time2;
    float Error2;
    float D;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        imu_IMU = hardwareMap.get(IMU.class, "imu");
        M1 = hardwareMap.get(DcMotor.class, "M1");
        M2 = hardwareMap.get(DcMotor.class, "M2");
        M3 = hardwareMap.get(DcMotor.class, "M3");
        M4 = hardwareMap.get(DcMotor.class, "M4");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        Servo1 = hardwareMap.get(Servo.class, "Servo1");
        M5AsDcMotor = hardwareMap.get(DcMotor.class, "M5AsDcMotor");
        M6AsDcMotor = hardwareMap.get(DcMotor.class, "M6AsDcMotor");

        Robot_Setup();
        waitForStart();
        ForwardTime(0.8, 1.48);
        TurnRight(0.5, 45);
        junction_UP(High_junction);
        ForwardTime(0.5, 0.4);
        Drop();
        BackwardTime(0.5, 0.5);
        turnLeft(0.5, 130);
        junction_DOWN(High_junction - ConeStack);
        ForwardTime(0.5, 1.8);
        PickUP();
        BackwardTime(0.5, 0.2);
        sleep(300);
        junction_UP(Low_junction);
        BackwardTime(0.5, 1.2);
        TurnRight(0.5, 110);
        junction_UP(High_junction - Low_junction);
        ForwardTime(0.5, 0.5);
        Drop();
        BackwardTime(0.5, 0.3);
        junction_DOWN(High_junction);
        TurnRight(0.5, 56);
        Mode2();
    }

    /**
     * Describe this function...
     */
    private void Robot_Setup() {
        int Medium_junction;

        ConeStack = 200;
        Low_junction = 1440;
        Medium_junction = 2440;
        High_junction = 3500;
        PickUP();
    }

    /**
     * Describe this function...
     */
    private void ForwardTime(double speed, double Timer) {
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

    /**
     * Describe this function...
     */
    private void TurnRight(double speed, int angles) {
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

    /**
     * Describe this function...
     */
    private void turnLeft(double speed, int angles) {
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

    /**
     * Describe this function...
     */
    private void BackwardTime(double speed, double Timer) {
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

    /**
     * Describe this function...
     */
    private void Mode1() {
        ForwardTime(0.8, 0.5);
    }

    /**
     * Describe this function...
     */
    private void Mode2() {
        BackwardTime(0.5, 1.2);
    }

    /**
     * Describe this function...
     */
    private void Drop() {
        Servo1.setPosition(-0.2);
    }

    /**
     * Describe this function...
     */
    private void PickUP() {
        Servo1.setPosition(0.55);
        sleep(500);
    }

    /**
     * Describe this function...
     */
    private void do_something() {
        imu_IMU.resetYaw();
        while (opModeIsActive()) {
            Angle = imu_IMU.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            telemetry.addData("Degrees", Angle.firstAngle);
            telemetry.update();
        }
    }

    /**
     * Describe this function...
     */
    private void junction_DOWN(int junction) {
        M5AsDcMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        M6AsDcMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        M5AsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        M6AsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        M5AsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        M6AsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        M5AsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        M6AsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        M5AsDcMotor.setTargetPosition(junction);
        M6AsDcMotor.setTargetPosition(junction);
        M5AsDcMotor.setPower(1);
        M6AsDcMotor.setPower(1);
        while (!(!M5AsDcMotor.isBusy() && !M6AsDcMotor.isBusy() && opModeIsActive())) {
        }
        M5AsDcMotor.setPower(0);
        M6AsDcMotor.setPower(0);
        sleep(500);
    }

    /**
     * Describe this function...
     */
    private void junction_UP(int junction) {
        M5AsDcMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        M6AsDcMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        M5AsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        M6AsDcMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        M5AsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        M6AsDcMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        M5AsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        M6AsDcMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        M5AsDcMotor.setTargetPosition(junction);
        M6AsDcMotor.setTargetPosition(junction);
        M5AsDcMotor.setPower(1);
        M6AsDcMotor.setPower(1);
        while (!(!M5AsDcMotor.isBusy() && !M6AsDcMotor.isBusy() && opModeIsActive())) {
        }
        M5AsDcMotor.setPower(0);
        M6AsDcMotor.setPower(0);
    }
}
