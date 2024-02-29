package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.roadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadRunner.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;

public class TrajectoryBuilder {
    public AutoBase.Coordinates c;
    public SampleMecanumDrive drive;

    public TrajectoryBuilder(AutoBase.Coordinates coordinates, SampleMecanumDrive drive){
        this.c = coordinates;
        this.drive = drive;

        trajectorySequenceLeft = computeTrajectory(propLocations.LEFT, drive, c);
        trajectorySequenceCenter = computeTrajectory(propLocations.CENTER, drive, c);
        trajectorySequenceRight = computeTrajectory(propLocations.RIGHT, drive, c);
    }

    public ArrayList<TrajectorySequence> computeTrajectory(propLocations propLoc, SampleMecanumDrive drive, AutoBase.Coordinates c) {
        ArrayList<TrajectorySequence> finalTrajectory = new ArrayList<>();

        Vector2d prepareFarDrop = c.prepareFarDrop;
        Vector2d backdropIntermediateFar = c.backdropIntermediateFar;
        Vector2d intermediateDropFar = c.intermediateDropFar;

        if (propLoc == propLocations.LEFT) {
            teamPropCoordinate = c.leftTeamProp;
            backdropCoordinate = c.backdropLeft;
            backdropIntermediateCoordinate = c.backdropIntermediateLeft;
        }
        else if (propLoc == propLocations.RIGHT) {
            teamPropCoordinate = c.rightTeamProp;
            backdropCoordinate = c.backdropRight;
            backdropIntermediateCoordinate = c.backdropIntermediateRight;
        }
        else {
            teamPropCoordinate = c.centerTeamProp;
            backdropCoordinate = c.backdropCenter;
            backdropIntermediateCoordinate = c.backdropIntermediateCenter;
        }

        assert drive != null;
        TrajectorySequence purpleDrop = drive.trajectorySequenceBuilder(c.startPose)
                .lineToLinearHeading(teamPropCoordinate)
                .build();
        finalTrajectory.add(purpleDrop);

        TrajectorySequence setupForBackdropNear = drive.trajectorySequenceBuilder(purpleDrop.end())
                //.forward(6)
                .lineTo(c.setupForBackdrop)
                .build();

        TrajectorySequence setupForBackdropFar = drive.trajectorySequenceBuilder(purpleDrop.end())
                .lineTo(prepareFarDrop)
                .lineTo(backdropIntermediateFar)
                .build();

        Pose2d endPurpledrop;
        if (c.isNearSide) {
            finalTrajectory.add(setupForBackdropNear);
            endPurpledrop = setupForBackdropNear.end();
        } else{
            finalTrajectory.add(setupForBackdropFar);
            endPurpledrop = setupForBackdropFar.end();
        }

        TrajectorySequence goToBackdropNear = drive.trajectorySequenceBuilder(endPurpledrop)
                //.lineToLinearHeading(backdropIntermediateCoordinate)
                .lineToLinearHeading(backdropCoordinate)
                .build();

        TrajectorySequence gotoBackdropFar = drive.trajectorySequenceBuilder(setupForBackdropFar.end())
                .lineTo(intermediateDropFar)
                .lineToLinearHeading(backdropIntermediateCoordinate)
                .lineToLinearHeading(backdropCoordinate)
                .build();

        if (c.isNearSide) {
            finalTrajectory.add(goToBackdropNear);
        }else{
            finalTrajectory.add(gotoBackdropFar);
        }

        TrajectorySequence parkRight = drive.trajectorySequenceBuilder(goToBackdropNear.end())
                //.forward(8)
                //.lineToLinearHeading(c.parkIntermediate)
                //.lineToLinearHeading(c.parkBetweenBackdrops)
                .lineTo(c.parkInCorner)
                .build();

        TrajectorySequence parkLeft = drive.trajectorySequenceBuilder(gotoBackdropFar.end())
                .lineTo(c.parkBetweenBackdrops)
                .build();

        if (c.isNearSide) {
            finalTrajectory.add(parkRight);
        }else{
            finalTrajectory.add(parkLeft);
        }

        return finalTrajectory;
    }

    public enum propLocations{
        LEFT,
        CENTER,
        RIGHT
    }


    public ArrayList<TrajectorySequence> trajectorySequenceLeft, trajectorySequenceCenter, trajectorySequenceRight;


    Pose2d teamPropCoordinate;
    Pose2d backdropCoordinate;
    Pose2d backdropIntermediateCoordinate;

}
