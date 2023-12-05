package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class BrookArm23_24 extends LinearOpMode {

    private PIDController controller;
    private PIDController controller2;
    public static double p = 0.01, i = 0.01, d = 0.0005;
    public static double f = 0.01;

    public static int target = -20;
    public static int target2 = -30;

    private final double ticksInDegree = 700/180.0;

    private DcMotor elbow;
    private DcMotor armbase;
    private DcMotor backright;
    private DcMotor frontright;
    private DcMotor backleft;
    private DcMotor frontleft;
    private Servo wrist;

    private Servo fingers;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        // Put initialization blocks here.
        waitForStart();
        if (opModeIsActive()) {
            double Speed;

            controller = new PIDController(p, i, d);
            controller2 = new PIDController(0.1, 0.01, 0.005);

            elbow = hardwareMap.get(DcMotor.class, "elbow");
            armbase = hardwareMap.get(DcMotor.class, "arm base");
            backright = hardwareMap.get(DcMotor.class, "back right");
            frontright = hardwareMap.get(DcMotor.class, "front right");
            backleft = hardwareMap.get(DcMotor.class, "back left");
            frontleft = hardwareMap.get(DcMotor.class, "front left");
            fingers = hardwareMap.get(Servo.class, "hand");
            wrist = hardwareMap.get(Servo.class, "wrist");

            backright.setDirection(DcMotor.Direction.REVERSE);
            frontright.setDirection(DcMotor.Direction.REVERSE);
            telemetry.update();
            Speed = 1;
            while (opModeIsActive()) {
                controller.setPID(p,i,d);
                controller2.setPID(0.1,0.01,0);

                int armPos = armbase.getCurrentPosition();
                double pid = controller.calculate(armPos, target);
                double ff = Math.cos(Math.toRadians(target/ticksInDegree)) * f;

                double power = pid + ff;

                armbase.setPower(power);

                int elPos = elbow.getCurrentPosition();
                double pid2 = controller2.calculate(elPos, target2);
                double ff2 = Math.cos(Math.toRadians(target2/ticksInDegree)) * f;

                double power2 = pid2 + ff2;

                elbow.setPower(power2);

                backleft.setPower(Speed * gamepad1.left_stick_y);
                frontleft.setPower(Speed * gamepad1.left_stick_y);
                backright.setPower(Speed * gamepad1.right_stick_y);
                frontright.setPower(Speed * gamepad1.right_stick_y);
                if (gamepad1.right_bumper) {
                    backright.setPower(Speed * -1);
                    frontright.setPower(Speed * 1);
                    backleft.setPower(Speed * -1);
                    frontleft.setPower(Speed * 1);
                } else if (gamepad1.left_bumper) {
                    backright.setPower(Speed * 1);
                    frontright.setPower(Speed * -1);
                    backleft.setPower(Speed * 1);
                    frontleft.setPower(Speed * -1);
                }
                if (gamepad1.a) {
                    Speed = 0.75;
                } else if (gamepad1.b) {
                    Speed = 0.5;
                } else if (gamepad1.x) {
                    Speed = 1;
                }
                // Below add gamepad #2
                if (gamepad2.dpad_up) {
                    target += 1;
                } else if (gamepad2.dpad_down) {
                    target -= 1;
                }else if (gamepad2.dpad_right) {
                    target2 += 1;
                } else if (gamepad2.dpad_left) {
                    target2 -= 1;
                }else if (gamepad2.dpad_left) {
                    target2 -= 1;
                }
                if (gamepad2.a) {
                    fingers.setPosition(0);
                }else if (gamepad2.b) {
                   fingers.setPosition(0.2);
                }
                if (gamepad2.right_bumper) {
                    wrist.setPosition(wrist.getPosition()+0.1);
                } else if (gamepad2.left_bumper) {
                    wrist.setPosition(wrist.getPosition()-0.1);
                }
            }
        }
    }
}
