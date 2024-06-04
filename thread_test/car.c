#include "car.h"
#include "mpu6050.h"
#include "LC_307.h"

//车轮PID
static Pos_PID_t L_pid_velocity;

static Pos_PID_t angle_con;
static Pos_PID_t speed_con;

void car_init(){

    Pos_PID_Init(&L_pid_velocity,1,0.5,0);
    L_pid_velocity.Output_Max = 50;
    L_pid_velocity.Output_Min = -50;
    L_pid_velocity.Value_I_Max = 200;
    L_pid_velocity.Ref = 0;

    Pos_PID_Init(&angle_con,1,0.5,0);
    angle_con.Output_Max = 50;
    angle_con.Output_Min = -50;
    angle_con.Value_I_Max = 200;
    angle_con.Ref = 0;

    Pos_PID_Init(&speed_con,1,0.5,0);
    speed_con.Output_Max = 50;
    speed_con.Output_Min = -50;
    speed_con.Value_I_Max = 200;
    speed_con.Ref = 0;

    init_wheel_codec();
    init_car_drive();
    car_stop();
    LC_307_Init();
    MPU6050Init();
    
}

void car_run(){
    
        

    //得到编码器速度值
    int left_speed = get_encoder_left();
    int right_speed = get_encoder_right();
    //计算电机 PID
    int left_pid = Pos_PID_Controller(&L_pid_velocity,left_speed);
    int right_pid = Pos_PID_Controller(&L_pid_velocity,right_speed);
    //控制电机转速
    car_drive(0,left_pid);
    car_drive(1,right_pid);

    usleep(40000); // 10ms delay
}