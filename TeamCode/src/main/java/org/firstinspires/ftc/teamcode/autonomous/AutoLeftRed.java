package org.firstinspires.ftc.teamcode.autonomous;

// imports

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.roadRunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.teamProp.TeamPropDetection;
import org.firstinspires.ftc.teamcode.tools.AutoDataStorage;
import org.firstinspires.ftc.teamcode.tools.Robot;


@Autonomous(name="Autonomous Left Red")
public class AutoLeftRed extends AutoBase {
    void Setup() {
        c = new Coordinates(false, false);
    }
}
