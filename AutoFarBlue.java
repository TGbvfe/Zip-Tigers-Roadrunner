package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "FarBlue", group = "examples")
public class AutoFarBlue extends LinearOpMode {

    public Servo Grasp;
    //public Servo armAngle;
    public Servo plane;
    public DcMotor armMotor;

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        armMotor = hardwareMap.dcMotor.get("armMotor");
        Grasp = hardwareMap.get(Servo.class,"Grasp");
        //armAngle = hardwareMap.get(Servo.class, "armAngle");
        plane = hardwareMap.get(Servo.class, "plane");

        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d(-37.60, 67.80, Math.toRadians(270.00)))
                .splineTo(new Vector2d(-56.59, 28.41), Math.toRadians(-85.13))
                .splineTo(new Vector2d(-46.00, 2.88), Math.toRadians(9.46))
                .splineTo(new Vector2d(56.28, 16.11), Math.toRadians(59.04))




                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.setPoseEstimate(traj.start());
        drive.followTrajectorySequence(traj);
    }
}
