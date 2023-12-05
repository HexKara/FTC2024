package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp
public class ArmTest extends OpMode{
    private PIDController controller;
    public static double p = 0.0001, i = 0.01, d = 0.005;
    public static double f = 0;

    public static int target = -10;

    private final double ticksInDegree = 700/180.0;

    private DcMotorEx armMotor;

    private DcMotor frontright;
    private DcMotor frontleft;
    private DcMotor backright;
    private DcMotor backleft;


    @Override
    public void  init(){
        FtcDashboard dashboard = FtcDashboard.getInstance();
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        armMotor = hardwareMap.get(DcMotorEx.class, "elbow");
        backright = hardwareMap.get(DcMotor.class, "back right");
        frontright = hardwareMap.get(DcMotor.class, "front right");
        backleft = hardwareMap.get(DcMotor.class, "back left");
        frontleft = hardwareMap.get(DcMotor.class, "front left");
    }

    @Override
    public void loop(){
        controller.setPID(p,i,d);
        int armPos = armMotor.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target/ticksInDegree)) * f;

        double power = pid + ff;

        armMotor.setPower(power);

        frontright.setDirection(DcMotorSimple.Direction.REVERSE);
        backleft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontright.setPower(gamepad1.right_stick_y);
        backright.setPower(gamepad1.right_stick_y);
        frontleft.setPower(gamepad1.left_stick_y);
        backleft.setPower(gamepad1.left_stick_y);

        telemetry.addData("pos ", armPos);
        telemetry.addData("target ", target);
        telemetry.update();
    }
}