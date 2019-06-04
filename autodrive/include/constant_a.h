#pragma once
//canny
#define canny_threshold_1 100
#define canny_threshold_2 100
#define apertureSize 3

//main
#define Delay 1000
#define ESC 27

// slope
#define Max_slope_degree 50
#define Min_slope_degree 40
#define stop_degree 3
#define inclination_standard 0
#define line_thickness 3

//fitline
#define fit_param 0
#define fit_reps 0.01
#define fit_aeps 0.01

//ŽëÇ¥Á¡
#define radius 2

//houghLineP
#define rho 1
#define delta CV_PI / 180.0
#define hough_threshold 30
#define MinLineLength 50
#define MaxLineGap 20

//addWeighted
#define alpha 1
#define beta 1
#define gamma 0

//¿ÞÂÊÀÎÁö ¿Àž¥ÂÊÀÎÁö Á€ÁöÀÎÁö ±žºÐ
#define Left 0
#define Right 1
#define Stop 2

// deque 
#define specimen 20
#define one_line_specimen 10

// ÈŸŽÜºžµµ val
#define MIN_val 98765432
#define MAX_val 0