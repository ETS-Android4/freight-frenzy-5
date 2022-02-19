package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class MatchState {
    public enum Alliance {
        RED,
        BLUE
    }
    public enum AutoPosition {
        WAREHOUSE,
        DUCK_SPIN
    }
    public static Pose2d AutoTransitionPose = new Pose2d();
    public static Alliance CurrentAlliance;
    public static AutoPosition CurrentPosition;
}
