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

        Pose2d backdropYellowCoordinate;

        Pose2d stackSetup;
        Pose2d stackLoc;


        if(propLoc == PROP_LOCATIONS.LEFT){
            backdropYellowCoordinate = c.backdropLeft;
        }

        if(propLoc == PROP_LOCATIONS.CENTER){
            backdropYellowCoordinate = c.backdropCenter;
        }

        if(propLoc == PROP_LOCATIONS.RIGHT){
            backdropYellowCoordinate = c.backdropRight;
        }

        if(config.truss_location == AutoBase.TRUSS_LOCATION.DOOR){
            stackSetup = c.rightStackSetup;
            stackLoc = c.rightStack;
        }
        if(config.truss_location == AutoBase.TRUSS_LOCATION.CENTER){
            stackSetup = c.centerStackSetup;
            stackLoc = c.centerStack;
        }
        if(config.truss_location == AutoBase.TRUSS_LOCATION.OUTER){
            stackSetup = c.leftStackSetup;
            stackLoc = c.leftStack;
        }

        // Trajectories
        TrajectorySequence purpleDrop;
        TrajectorySequenceBuilder purpleDropBuilder = drive.trajectorySequenceBuilder(c.startPose);

        TrajectorySequence intake1;
        TrajectorySequenceBuilder intake1Builder = drive.trajectorySequenceBuilder(null); // placeholder
        intake1Builder
                .lineTo(stackSetup.vec())




        // Purple drop


                        .lineToLinearHeading(c.backdropSideRightPurpleCoordinateD);


        // backdrop side purple dropOff
        if(config.side == AutoBase.SIDE.NEAR){
            if(propLoc == PROP_LOCATIONS.LEFT){
                purpleDropBuilder
                        .lineTo(c.audienceSideLeftPurpleToRightStackCoordinateA)
                        .lineToLinearHeading(c.audienceSideLeftPurpleToRightStackCoordinateB)
                        .lineToLinearHeading(c.rightStackSetup);
            }

            if(propLoc == PROP_LOCATIONS.CENTER){
                purpleDropBuilder
                        .lineToLinearHeading(c.audienceSideMiddlePurpleToRightStackCoordinateA)
                        .lineTo(c.audienceSideMiddlePurpleToRightStackCoordinateB)
                        .lineToLinearHeading(c.rightStackSetup);
            }

            if(propLoc == PROP_LOCATIONS.RIGHT){
                purpleDropBuilder
                        .lineToLinearHeading(c.audienceSideRightPurpleToRightStackCoordinateA)
                        .splineTo(c.audienceSideRightPurpleToRightStackCoordinateBVector, c.audienceSideRightPurpleToRightStackCoordinateBAngle)
                        .splineTo(c.rightStackSetup.vec(), c.rightStackSetup.getHeading());
            }
        }

        // audience side purple dropOff
        else {
            if(propLoc == PROP_LOCATIONS.LEFT){
                purpleDropBuilder
                        .lineToLinearHeading(c.backdropSideLeftPurpleCoordinateA)
                        .lineToLinearHeading(c.backdropSideLeftPurpleCoordinateB);
            }

            if(propLoc == PROP_LOCATIONS.CENTER){
                purpleDropBuilder
                        .lineTo(c.backdropSideCenterPurpleCoordinateA)
                        .lineToLinearHeading(c.backdropSideCenterPurpleCoordinateB);
            }

            if(propLoc == PROP_LOCATIONS.RIGHT){
                purpleDropBuilder
                        .lineTo(c.backdropSideRightPurpleCoordinateA)
                        .lineToLinearHeading(c.backdropSideRightPurpleCoordinateB)
                        .lineTo(c.backdropSideRightPurpleCoordinateC);
            }
        }
        purpleDrop = purpleDropBuilder.build();
        finalTrajectory.add(purpleDrop);


        // backdrop side code
        if(config.side == AutoBase.SIDE.NEAR){
            TrajectorySequence backdropSideYellowDrop = drive.trajectorySequenceBuilder(finalTrajectory.get(finalTrajectory.size() - 1).end())
                    .lineToLinearHeading(backdropYellowCoordinate)
                    .build();
            finalTrajectory.add(backdropSideYellowDrop);
        }

        // audience side code
        else{
            intake1Builder.setStartPose(finalTrajectory.get(finalTrajectory.size() - 1).end());
            intake1 = intake1Builder.build();
            finalTrajectory.add(intake1);
        }



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
