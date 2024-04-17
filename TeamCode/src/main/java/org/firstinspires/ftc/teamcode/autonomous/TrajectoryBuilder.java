package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.roadRunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadRunner.trajectorysequence.TrajectorySequenceBuilder;

import java.util.ArrayList;

public class TrajectoryBuilder {
    AutoBase.ConfigItems config;
    private final AutoBase.Coordinates c;
    private final SampleMecanumDrive drive;

    public enum PROP_LOCATIONS {
        LEFT,
        CENTER,
        RIGHT
    }


    public TrajectoryBuilder(AutoBase.Coordinates coordinates, SampleMecanumDrive drive, AutoBase.ConfigItems config){
        this.config = config;
        this.c = coordinates;
        this.drive = drive;

        trajectorySequenceLeft = computeTrajectory(PROP_LOCATIONS.LEFT);

        trajectorySequenceCenter = computeTrajectory(PROP_LOCATIONS.CENTER);

        trajectorySequenceRight = computeTrajectory(PROP_LOCATIONS.RIGHT);
    }

    public ArrayList<TrajectorySequence> computeTrajectory(PROP_LOCATIONS propLoc) {
        ArrayList<TrajectorySequence> finalTrajectory = new ArrayList<>();

        TrajectorySequenceBuilder purpleDrop = drive.trajectorySequenceBuilder(c.startPose);

        // near side purple dropOff
        if(config.side == AutoBase.SIDE.NEAR){
            if(propLoc == PROP_LOCATIONS.RIGHT){
                purpleDrop
                        .lineToLinearHeading(new Pose2d(-41.01, 21.65, Math.toRadians(0.00)))
                        .splineTo(new Vector2d(-35.82, 13.35), Math.toRadians(270.00))
                        .splineTo(c.rightStackSetup.vec(), Math.toRadians(180.00));
            }

        }



        // AUDIENCE SIDE PURPLE
        // START POSE: new Pose2d(12.00, 62.00, Math.toRadians(90.00))

        TrajectorySequence audienceRightPurpleToRightStack = drive.trajectorySequenceBuilder(c.startPose)
                .lineToLinearHeading(new Pose2d(-41.01, 21.65, Math.toRadians(0.00)))
                .splineTo(new Vector2d(-35.82, 13.35), Math.toRadians(270.00))
                .splineTo(c.rightStackSetup.vec(), Math.toRadians(180.00))
                .build();



        TrajectorySequence audienceMiddlePurpleToRightStack = drive.trajectorySequenceBuilder(c.startPose)
                .lineToLinearHeading(new Pose2d(-40.50, 24.70, Math.toRadians(180.00)))
                .lineTo(new Vector2d(-44.50, 24.70))
                .lineToLinearHeading(c.rightStackSetup)
                .build();

        TrajectorySequence audienceLeftPurpleToRightStack = drive.trajectorySequenceBuilder(new Pose2d(-36.00, 62.00, Math.toRadians(90.00)))
                .lineTo(new Vector2d(-36.00, 42.00))
                .lineToLinearHeading(new Pose2d(-32.73, 31.07, Math.toRadians(180.00)))
                .lineToLinearHeading(c.rightStackSetup)
                .build();




        // BACKDROP SIDE PURPLE
        // START POSE: new Pose2d(12.00, 62.00, Math.toRadians(90.00))

        TrajectorySequence backdropSideLeftPurple = drive.trajectorySequenceBuilder(c.startPose)
                .lineToLinearHeading(new Pose2d(16.00, 29.00, Math.toRadians(180.00)))
                .lineToLinearHeading(new Pose2d(13.00, 29.00, Math.toRadians(180.00)))
                .lineToLinearHeading(new Pose2d(13.00, 45.00, Math.toRadians(180.00)))
                .build();

        TrajectorySequence backdropSideCenterPurple = drive.trajectorySequenceBuilder(c.startPose)
                .lineTo(new Vector2d(10.50, 31.50))
                .lineToLinearHeading(new Pose2d(15.50, 38.50, Math.toRadians(90.00)))
                .lineToLinearHeading(new Pose2d(16.00, 38.50, Math.toRadians(180.00)))
                .build();

        TrajectorySequence backdropSideRightPurple = drive.trajectorySequenceBuilder(c.startPose)
                .lineTo(new Vector2d(12.00, 42.00))
                .lineToLinearHeading(new Pose2d(8.23, 33.81, Math.toRadians(0.00)))
                .lineTo(new Vector2d(15.23, 33.81))
                .lineToLinearHeading(new Pose2d(15.73, 33.81, Math.toRadians(180.0)))
                .build();

        TrajectorySequence parkRight = drive.trajectorySequenceBuilder(goToBackdrop.end())
                .lineTo(c.parkInCorner)
                .back(10)
                .build();

        TrajectorySequence parkLeft = drive.trajectorySequenceBuilder(goToBackdrop.end())
                .forward(2)
                .build();

        if (config.side == AutoBase.SIDE.NEAR) {
            finalTrajectory.add(parkRight);
        }else{
            finalTrajectory.add(parkLeft);
        }
*/
        return finalTrajectory;
    }


    public ArrayList<TrajectorySequence> trajectorySequenceLeft, trajectorySequenceCenter, trajectorySequenceRight;


    Pose2d teamPropCoordinate;
    Pose2d backdropCoordinate;
    Pose2d backdropStrafeCoordinate;

}
