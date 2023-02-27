package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class Specifications
{
    public static float moving_speed_teleop = 0.4f;
    public static float arm_moving_speed_up_auto = 0.45f;
    public static float glis_min_speed = 0.2f;
    public static float glis_speed_multiplier = 1.0f;
    public static float crem_speed_auto = 1f;


    public static int upper_limit_glis_ticks = 57000;
    public static int glis_180_space_ticks = 4700;
    public static int glis_midj_ticks = 6500;
    public static int glis_highj_ticks = 9400;



    public static float servo_claw_pos_up = 1f;
    public static float servo_claw_pos_down = 0f;

    public static float servo_180_pos_up = 1f;
    public static float servo_180_pos_down = 0.1f;
    public static float servo_180_auto_drop = 0.4f;



    public static float servo_forebar_pos_up = 0f;
    public static float servo_forebar_pos_down = 1f;
    public static float servo_forbar_start_auton = 0.35f;

    public static float servo_movement_speed = 1;



    // buna!!
    public static float movement_speed_fast = 0.6f;
    public static float movement_speed_slow = 0.2f;


    public static float crem_speed = 0.5f;

    public static int lower_limit_crem_ticks = -1250; //-1372 max
    public static int upper_limit_crem_ticks = 700;  //995 max


    public static PIDFCoefficients newPIDF = new PIDFCoefficients(12.5, 3.0, 0.0, 30.0);
    public static PIDFCoefficients runToPosPIDF = new PIDFCoefficients(1, 0, 0.0, 0);

    public static float glis_speed_extension = 1f;
    public static float glis_speed_fast_retraction = 0.15f;
    public static float glis_speed_min_retraction = 0.01f;
    public static float glis_speed_stationary = 0.1f;
    public static float glis_speed_threshold_multi = 0.00008f;

    public static int glis_threshold_gradual_slowdown = 1500;
    public static int glis_threshold_stationary = 100;
    public static int glis_treshold_move = 720;

    public static int glis_high_ticks = 2100; //2300 max
    public static int glis_mid_ticks = 1440;

    public static float servo_claw_open = 0.85f;
    public static float servo_claw_closed = 0f;

    public static float servo_forebar_up = 0.2f;
    public static float servo_forebar_down =0.8f;

    public static float servo_180_up = 1f;
    public static float servo_180_down = 0f;

    public static float servo_movement_ms = 1f;


    public static int button_delay = 200;

    public static float delayAutonAction = 4000;
}