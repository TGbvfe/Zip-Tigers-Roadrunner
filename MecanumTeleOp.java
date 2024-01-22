package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@TeleOp(name = "mecanum", group = "examples")
public class MecanumTeleOp extends LinearOpMode {

    public DcMotor rightRear;
    public DcMotor leftRear;
    public DcMotor rightFront;
    public DcMotor leftFront;
    public Servo Grasp;
    public Servo plane;
    public CRServo armAngle;
    public DcMotor armMotor;

    @Override
    public void runOpMode() throws InterruptedException{
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);


        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftRear = hardwareMap.dcMotor.get("leftRear");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightRear = hardwareMap.dcMotor.get("rightRear");
        armMotor = hardwareMap.dcMotor.get("armMotor");

        Grasp = hardwareMap.get(Servo.class, "Grasp");
        armAngle = hardwareMap.get(CRServo.class, "armAngle");
        plane = hardwareMap.get(Servo.class, "plane");

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()){

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;

            double turn = gamepad1.right_stick_x;

            double armJoy = -gamepad2.left_stick_y;

            double armAngleJoy = gamepad1.right_stick_y;

            if (armJoy != 0) {
                armMotor.setPower(armJoy / 1.25);
            }
            else{
                armMotor.setPower(0.2);
            }
            if (gamepad2.x){
                plane.setPosition(0);
            }
            if (gamepad2.b) {
                plane.setPosition(1);
            }

           armAngle.setPower(armAngleJoy);

            if (gamepad2.a){
                Grasp.setPosition(0.4);
            }
            if (gamepad2.y){
                Grasp.setPosition(0.1);
            }

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(turn), 1);

            leftFront.setPower( (y + x + turn) / denominator);
            leftRear.setPower( (y - x + turn) / denominator);
            rightFront.setPower( (y - x - turn) / denominator);
            rightRear.setPower( (y + x - turn) / denominator);





        }
    }




}