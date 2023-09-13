/*!
 * @author LiNY2 
 * @date 2023/03/27 0027
 */


#include "ctrlPathTracker.h"


PathTracker::PathTracker(BaseChassis *chassisPtr) : chassis_ptr(chassisPtr) {
    go2Point = new Go2Point(chassisPtr);
}
PathTracker::~PathTracker()
{
    delete go2Point;
}

void PathTracker::SetPathPoints(const PlanPoint *points, uint16_t point_num)
{
    path_point = points;
    PathTracker::point_num = point_num;
    point_index = 1 ; ///< 使now_pos2next_target为非零值
    status_flag = Running;

}
void PathTracker::SetyawPID(float kp, float ki, float kd)
{
    yawPID.Kp = kp;
    yawPID.Kd = kd;
    yawPID.Ki = ki;
}
void PathTracker::SetnormalCorrPID_x(float kp, float ki, float kd)
{
    normalCorrPID_x.Kp = kp;
    normalCorrPID_x.Kd = kd;
    normalCorrPID_x.Ki = ki;
}
void PathTracker::SetnormalCorrPID_y(float kp, float ki, float kd)
{
    normalCorrPID_y.Kp = kp;
    normalCorrPID_y.Kd = kd;
    normalCorrPID_y.Ki = ki;
}
void PathTracker::ResetStatus()
{
    path_point = nullptr;
    PathTracker::point_num = 0;
    point_index = 0;
    status_flag = Reset;
    PID_Reset(&yawPID);
    PID_Reset(&normalCorrPID_x);
    PID_Reset(&normalCorrPID_y);
    go2Point->ResetStatus();
    chassis_ptr->Chassis_Stop();
}
uint8_t PathTracker::UpdatePointIndex(PostureStatus_s nowpos,CMD_Vel_t nowcmd)
{
    if(point_index >= point_num - 1)
    {
        return 0;
    }
    ///< 底盘当前速度向量
    vec forehead =  Vec_Create(cos(nowcmd.dir), sin(nowcmd.dir)); ///< 当前期望速度方向向量
    vec now_speed_vec = Vec_Create(nowpos.speed_x, nowpos.speed_y); ///< 底盘当前的速度向量
    vec now_pos2now_target = Vec_Create(path_point[point_index].x - nowpos.x ,
                                    path_point[point_index].y - nowpos.y); ///< 底盘当前坐标到当前目标点的位移向量
    vec now_pos2next_target = Vec_Create(path_point[point_index + 1].x - nowpos.x,
                                     path_point[point_index + 1].y - nowpos.y); ///<底盘到下一个目标点的位移向量
    float distance_to_next = Vec_Model(now_pos2now_target);///< 底盘到当前目标点的距离
    if(
        Vec_DotProduct(now_pos2now_target,now_pos2next_target) < 0 ||
        Vec_DotProduct(now_pos2now_target,forehead) < 0 ||
        distance_to_next <= passedWaypointCircleTh
    )
    {
        point_index++;
        return 1;
    }
    return 0;
}
void PathTracker::Ctrl_exe()
{
    if(!status_flag)
    {
        return ;
    }
    ///< 更新目标点，若更新到最后一个点，转移状态。
    uint8_t updateIdxRet = 0;
    uint16_t last_index = point_index;
    auto nowpos=chassis_ptr->Chassis_GetPostureStatus();
    auto nowvel = chassis_ptr->Chassis_GettargetVel();
    while((updateIdxRet = UpdatePointIndex(nowpos,nowvel)) == 1)
        ;

    if(last_index < point_num - 1 && point_index >= point_num - 1 )
    {
        status_flag = InitGo2LastPt;
    }
    switch (status_flag)
    {
        case Running:
        {
            vec now_speed_vec = Vec_Create(nowpos.speed_x,nowpos.speed_y);
            vec target_speed_vec = Vec_Create((float)path_point[point_index].speed *cos(path_point[point_index].direct),
                                              (float)path_point[point_index].speed *sin(path_point[point_index].direct));
            vec now_target2last_target = Vec_Create(path_point[point_index - 1].x - path_point[point_index].x ,
                                                    path_point[point_index - 1].y - path_point[point_index].y);
            vec now_pos2now_target = Vec_Create(path_point[point_index].x - nowpos.x,
                                                path_point[point_index].y - nowpos.y);
            TrackVector(now_speed_vec,target_speed_vec,
                        now_target2last_target,now_pos2now_target,
                        nowpos.yaw,path_point[point_index].target_angle);
            break ;
        }
        case InitGo2LastPt:
        {
            Point2D_s last_pt{path_point[point_index].x,
                              path_point[point_index].y};
            go2Point->SetTarget(last_pt,path_point[point_index].target_angle,
                                sqrtf(nowpos.x*nowpos.x + nowpos.y*nowpos.y),0);
            status_flag = Go2LastPT;
            break;
        }
        case Go2LastPT:
        {
            go2Point->Ctrl_exe();
            if(go2Point->getStatusFlag() == Go2Point::Arrived)
            {
                status_flag = Arrived;
            }
            break;
        }
        case Arrived:
        {
            ///< 可以不动，也可以持续锁死在最后一点；
            chassis_ptr->Chassis_Stop();
            break ;
        }
        default:{break;}
    }

}
/**
 * @brief 计算法向修正向量，计算偏航角控制量。
 * @param now_speed_vec 当前速度
 * @param target_speed_vec 目标速度
 * @param now_target2last_target 当前目标点到上一目标点的向量
 * @param now_pos2now_target 位移向量
 * @param now_yaw 底盘当前偏航角
 * @param target_yaw 偏航角
 **/
void PathTracker::TrackVector(vec now_speed_vec, vec target_speed_vec, vec now_target2last_target, vec now_pos2now_target, float now_yaw,float target_yaw)
{
    if (Vec_IsZero(target_speed_vec))
    {
        chassis_ptr->Chassis_SetTargetVel(0,0,0,false,true,false);
        return;
    }

    float project = fabs(Vec_DotProduct(now_target2last_target,now_pos2now_target))/ Vec_Model(now_target2last_target);///<投影长度
    vec project_vec = Vec_ScalarMul(now_target2last_target,project / Vec_Model(now_target2last_target));///<投影向量
    vec corr_vec = Vec_Add(now_pos2now_target,project_vec); ///< 法向修正向量
//    Vec_ScalarMul(corr_vec,1.0); /// < 使距离修正与速度大小相关联
    float corr_vec_ctrl_x = -PID_GetOutput(&normalCorrPID_x, 0, corr_vec.x); // 需要使用两个PID结构体，否则last变量会被重用
    float corr_vec_ctrl_y = -PID_GetOutput(&normalCorrPID_y, 0, corr_vec.y);
    vec corr_vec_ctrl = Vec_Create(corr_vec_ctrl_x, corr_vec_ctrl_y);
    vec ctrl_speed_vec = Vec_Add(corr_vec_ctrl, target_speed_vec); ///< 法向修正速度和目标速度合成，得到最终输出速度

    // 偏航角控制
    float delta_angle = target_yaw - now_yaw;
    delta_angle = AngleLimitPI(delta_angle);
    target_yaw = now_yaw + delta_angle;
    float ctrl_omega = PID_GetOutput(&yawPID, target_yaw, now_yaw); ///< 暂时不加入角速度内环

    float ctrl_speed = (float)Vec_Model(ctrl_speed_vec);
    __LIMIT_FROM_TO(ctrl_speed, DRIVE_WHEEL_MIN_SPEED, DRIVE_WHEEL_MAX_SPEED);
    float ctrl_dir = atan2(ctrl_speed_vec.y, ctrl_speed_vec.x);

    chassis_ptr->Chassis_SetTargetVel(ctrl_speed,ctrl_dir,ctrl_omega);

}
