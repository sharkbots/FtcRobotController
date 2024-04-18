package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.roadRunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadRunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.tools.Robot;

import java.util.ArrayList;

public class TrajectoryBuilder {
    AutoBase.ConfigItems config;
    private final AutoBase.Coordinates c;
    private final SampleMecanumDrive drive;
    Robot robot;

    public enum PROP_LOCATIONS {
        LEFT,
        CENTER,
        RIGHT
    }


    public TrajectoryBuilder(AutoBase.Coordinates coordinates, SampleMecanumDrive drive, AutoBase.ConfigItems config, Robot robot){
        this.config = config;
        this.c = coordinates;
        this.drive = drive;
        this.robot = robot;

        trajectorySequenceLeft = computeTrajectory(PROP_LOCATIONS.LEFT);

        trajectorySequenceCenter = computeTrajectory(PROP_LOCATIONS.CENTER);

        trajectorySequenceRight = computeTrajectory(PROP_LOCATIONS.RIGHT);
    }

    public ArrayList<TrajectorySequence> computeTrajectory(PROP_LOCATIONS propLoc) {
        ArrayList<TrajectorySequence> finalTrajectory = new ArrayList<>();

        Pose2d backdropYellowCoordinate = null;

        Pose2d stackSetup = null;
        Pose2d stackLoc = null;


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

        TrajectorySequence goToBackdrop1;
        TrajectorySequenceBuilder goToBackdrop1Builder = drive.trajectorySequenceBuilder(null); // placeholder

        TrajectorySequence park;
        TrajectorySequenceBuilder parkBuilder = drive.trajectorySequenceBuilder(null); // placeholder



        // Purple drop

        // audience side purple dropOff
        if(config.side == AutoBase.SIDE.AUDIENCE){
            if(propLoc == PROP_LOCATIONS.LEFT){
                purpleDropBuilder
                        .lineTo(c.audienceSideLeftPurpleToRightStackCoordinateA)
                        .lineToLinearHeading(c.audienceSideLeftPurpleToRightStackCoordinateB)
                        .lineToLinearHeading(stackSetup);
            }

            if(propLoc == PROP_LOCATIONS.CENTER){
                purpleDropBuilder
                        .lineToLinearHeading(c.audienceSideMiddlePurpleToRightStackCoordinateA)
                        .lineTo(c.audienceSideMiddlePurpleToRightStackCoordinateB)
                        .lineToLinearHeading(stackSetup);
            }

            if(propLoc == PROP_LOCATIONS.RIGHT){
                purpleDropBuilder
                        .lineToLinearHeading(c.audienceSideRightPurpleToRightStackCoordinateA)
                        .splineTo(c.audienceSideRightPurpleToRightStackCoordinateB.vec(), c.audienceSideRightPurpleToRightStackCoordinateB.getHeading())
                        .splineTo(stackSetup.vec(), stackSetup.getHeading());
            }
        }

        // backdrop side purple dropOff
        else {
            if(propLoc == PROP_LOCATIONS.LEFT){
                purpleDropBuilder
                        .lineToLinearHeading(c.backdropSideLeftPurpleCoordinateA)
                        .lineToLinearHeading(c.backdropSideLeftPurpleCoordinateB)
                        .lineToLinearHeading(c.backdropSideLeftPurpleCoordinateC);
            }

            if(propLoc == PROP_LOCATIONS.CENTER){
                purpleDropBuilder
                        .lineTo(c.backdropSideCenterPurpleCoordinateA)
                        .lineToLinearHeading(c.backdropSideCenterPurpleCoordinateB)
                        .lineToLinearHeading(c.backdropSideCenterPurpleCoordinateC);
            }

            if(propLoc == PROP_LOCATIONS.RIGHT){
                purpleDropBuilder
                        .lineTo(c.backdropSideRightPurpleCoordinateA)
                        .lineToLinearHeading(c.backdropSideRightPurpleCoordinateB)
                        .lineTo(c.backdropSideRightPurpleCoordinateC)
                        .lineToLinearHeading(c.backdropSideRightPurpleCoordinateD);

            }
        }
        purpleDrop = purpleDropBuilder.build();
        finalTrajectory.add(purpleDrop); // index 0


        // backdrop side code
        if(config.side == AutoBase.SIDE.BACKDROP){
            TrajectorySequence backdropSideYellowDrop = drive.trajectorySequenceBuilder(finalTrajectory.get(finalTrajectory.size() - 1).end())
                    .lineToLinearHeading(backdropYellowCoordinate)
                    .build();
            finalTrajectory.add(backdropSideYellowDrop); // index 1 backdrop
        }

        // audience side code
        else{
            intake1Builder.setStartPose(finalTrajectory.get(finalTrajectory.size() - 1).end());
            intake1Builder.lineToLinearHeading(stackLoc);
            intake1 = intake1Builder.build();
            finalTrajectory.add(intake1); // index 1 audience


            if(config.stack_location == AutoBase.STACK_LOCATION.RIGHT){
                goToBackdrop1Builder.setStartPose(finalTrajectory.get(finalTrajectory.size() - 1).end());
                goToBackdrop1Builder
                        .lineToLinearHeading(c.prepareToGoToStageDoor)
                        .lineToLinearHeading(c.prepareToGoToStageDoor2)
                        .lineToLinearHeading(c.intermediateCyclePose)
                        .addTemporalMarker(2, ()-> robot.outTakeSetClawYawVertical.runAsync())
                        .lineToLinearHeading(c.prepareToGoToBackdropCycle);
                        //.splineTo(c.intermediateCyclePose.vec(), c.intermediateCyclePose.getHeading(), SampleMecanumDrive.getVelocityConstraint(30, 30, DriveConstants.TRACK_WIDTH),
                                //SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL));
                        //.addSpatialMarker((c.spatialMarkerGoToBackdrop), ()-> robot.autoOutTakeYellow.runAsync())
            }
            goToBackdrop1 = goToBackdrop1Builder.build();
            finalTrajectory.add(goToBackdrop1); // index 2 audience

            TrajectorySequence audienceSideYellowDrop = drive.trajectorySequenceBuilder(finalTrajectory.get(finalTrajectory.size() - 1).end())
                    .lineToLinearHeading(backdropYellowCoordinate)
                    .build();
            finalTrajectory.add(audienceSideYellowDrop); // index 3 audience
        }



        parkBuilder.setStartPose(finalTrajectory.get(finalTrajectory.size() - 1).end());

        if(config.park_location == AutoBase.PARK_LOCATION.CORNER){
            parkBuilder
                    .lineTo(c.parkInCornerSetup)
                    .lineTo(c.parkInCorner);
        }
        else if(config.park_location == AutoBase.PARK_LOCATION.BETWEEN_BACKDROPS){
            parkBuilder
                    .lineTo(c.parkBetweenBackdropsSetup)
                    .lineTo(c.parkBetweenBackdrops);
        }
        park = parkBuilder.build();
        finalTrajectory.add(park);



        return finalTrajectory;
    }


    public ArrayList<TrajectorySequence> trajectorySequenceLeft, trajectorySequenceCenter, trajectorySequenceRight;


    Pose2d teamPropCoordinate;
    Pose2d backdropCoordinate;
    Pose2d backdropStrafeCoordinate;

}
