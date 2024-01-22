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

@Autonomous(name = "CloseRed", group = "examples")
public class AutoMouse extends LinearOpMode {

    public Servo Grasp;
    //public Servo armAngle;
    public Servo plane;
    //public DcMotor armMotor;

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        //armMotor = hardwareMap.dcMotor.get("armMotor");
        Grasp = hardwareMap.get(Servo.class,"Grasp");
        //armAngle = hardwareMap.get(Servo.class, "armAngle");
        plane = hardwareMap.get(Servo.class, "plane");

        //armMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        TrajectorySequence traj = drive.trajectorySequenceBuilder(new Pose2d(12.84, -68.42, Math.toRadians(90.00)))
                .splineTo(new Vector2d(55.03, -61.10), Math.toRadians(175.49))



                .build();

        waitForStart();

        if (isStopRequested()) return;

        drive.setPoseEstimate(traj.start());
        drive.followTrajectorySequence(traj);
    }
}
