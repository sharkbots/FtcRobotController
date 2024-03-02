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

        Vector2d backdropIntermediateFar = c.backdropIntermediateFar;
        Vector2d intermediateDropFar = c.intermediateDropFar;

        if (propLoc == propLocations.LEFT) {
            teamPropCoordinate = c.leftTeamProp;
            backdropIntermediateCoordinate = c.backdropIntermediateLeft;
            backdropCoordinate = c.backdropLeft;
            backdropStrafeCoordinate = c.backdropCenter;
        }
        else if (propLoc == propLocations.RIGHT) {
            teamPropCoordinate = c.rightTeamProp;
            backdropIntermediateCoordinate = c.backdropIntermediateRight;
            backdropCoordinate = c.backdropRight;
            backdropStrafeCoordinate = c.backdropCenter;

        }
        else {
            teamPropCoordinate = c.centerTeamProp;
            backdropIntermediateCoordinate = c.backdropIntermediateCenter;
            backdropCoordinate = c.backdropCenter;
            backdropStrafeCoordinate = c.backdropLeft; // arbitrary left, could be right

        }

        assert drive != null;
        TrajectorySequence purpleDrop = drive.trajectorySequenceBuilder(c.startPose)
                .lineToLinearHeading(teamPropCoordinate)
                .build();
        finalTrajectory.add(purpleDrop); //get 0




        TrajectorySequence setupForBackdropNear = drive.trajectorySequenceBuilder(purpleDrop.end())
                .lineTo(c.setupForBackdrop)
                .build();

        TrajectorySequence setupForBackdropFar = drive.trajectorySequenceBuilder(purpleDrop.end())
                .lineTo(c.prepareFarDrop)
                .lineTo(backdropIntermediateFar)
                .build();

        Pose2d endPurpledrop;
        if (c.isNearSide) {
            finalTrajectory.add(setupForBackdropNear);
            endPurpledrop = setupForBackdropNear.end();
        } else{
            finalTrajectory.add(setupForBackdropFar);
            endPurpledrop = setupForBackdropFar.end();
        } // get 1

        TrajectorySequence goToBackdrop = drive.trajectorySequenceBuilder(endPurpledrop)
                .lineToLinearHeading(backdropCoordinate)
                .back(3)
                .build();

        finalTrajectory.add(goToBackdrop); // get 2

        TrajectorySequence parkRight = drive.trajectorySequenceBuilder(goToBackdrop.end())
                //.forward(8)
                //.lineToLinearHeading(c.parkIntermediate)
                //.lineToLinearHeading(c.parkBetweenBackdrops)
                .lineTo(c.parkInCorner)
                .back(10)
                .build();

        TrajectorySequence parkLeft = drive.trajectorySequenceBuilder(goToBackdrop.end())
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
    Pose2d backdropStrafeCoordinate;
    Pose2d backdropIntermediateCoordinate;

}
