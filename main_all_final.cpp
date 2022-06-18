//
// Created by yxt on 09/06/2022.
//
#include <iostream>
#include <franka/exception.h>
#include <franka/gripper.h>
#include <franka/robot.h>
#include <string>
#include "/home/yxt/catkin_ws/devel/include/tengapp/channels.h"
#include "ros/ros.h"
#include "examples_common.h"
#include <ctime>
#include <cstdlib>
#include <gtest/gtest.h>
#include <liborl/liborl.h>
#include "std_msgs/Int8.h"
#include <eigen3/Eigen/Eigen>


using namespace std;
using namespace orl;

class handle{
public:
    ros::NodeHandle node_handle;
    ros::Subscriber sub;
    ros::Subscriber sub_state;
    ros::Publisher pub_state;
    const int windows_size;
    int state;
    int count;
    float *buffer;
    float *buffer1;
    float *buffer2;
    float *buffer3;
    float *buffer4;
    float *buffer5;
    float *baseline;
    float step_dis;
    float max_tail;
    float reset_voltage;
    float grasp_width;
    float cutoff_min, cutoff_max;
    float cutoff_width;
    bool init_flag;
    bool trigger1, trigger2;
    int cnt;
    int curr_cnt;
    Eigen::Vector3d target1, target2;
    Eigen::Vector3d icurr_pos, ilast_pos;
    array<double, 7> target3; // {{1.02312 -0.563539 -0.637039 -2.56926 1.53146 2.20833 -1.39992}}
    const double execution_time;
    ros::Time begin_time;
    ros::Time end_time;
    bool reached_target1;
    bool reached_target2; // array<double, 7> tar_joint = {{1.05461, -0.181552, -0.482476, -2.37842, -0.120223, 2.18888, -0.076247}};
    handle():execution_time(3.0), windows_size(10){
        buffer = new float [windows_size];
        buffer1 = new float [windows_size];
        buffer2 = new float [windows_size]; // windows size 15
        buffer3 = new float  [windows_size*2];
        buffer4 = new float  [windows_size*2];
        buffer5 = new float  [windows_size*2];
        baseline = new float  [windows_size*4];

        trigger1 = false;
        trigger2 = false;
        state = 6;
        grasp_width = 0.04;
        count = -1;
        step_dis = -0.1;
        max_tail = 0.0;
        cutoff_min = 1.0;
        cutoff_max = 3;
        cutoff_width = cutoff_max - cutoff_min;
        reached_target1 = false;
        reached_target2 = false;
        init_flag = false;
        cnt = 0;
        curr_cnt = 0;

        memset(buffer, 0, windows_size*sizeof(*buffer));
        memset(buffer1, 0, windows_size*sizeof(*buffer1));
        memset(buffer2, 0, windows_size*sizeof(*buffer2));
        memset(buffer3, 0, 2*windows_size*sizeof(*buffer3));
        memset(buffer4, 0, 2*windows_size*sizeof(*buffer4));
        memset(buffer5, 0, 2*windows_size*sizeof(*buffer5));

        target1 = Eigen::Vector3d(0.5, -0.2, 0.3);
        target2 = Eigen::Vector3d(0.5, 0.1, 0.3);
        target3 = {{1.02312, -0.563539, -0.637039, -2.56926, 1.53146, 2.20833, -1.39992}};
        ilast_pos = Eigen::Vector3d(0.0, 0.0, 0.0);
        icurr_pos = Eigen::Vector3d(0.0, 0.0, 0.0);

        sub = node_handle.subscribe("TENG_SIGNALS", 10, &handle::Callback2, this);
        sub_state = node_handle.subscribe("state", 10, &handle::CallbackState, this);
        pub_state = node_handle.advertise<std_msgs::Int8>("state", 1000);
    }
    orl::Pose goal_pose = orl::Pose({0.1,-0.1,0}, {0,0,0});
    orl::Robot orl_robot = orl::Robot("192.168.5.12");
    void Callback(const tengapp::channels& msg); // receive signals
    void Callback2(const tengapp::channels& msg);
    void CallbackState(const std_msgs::Int8& msg); // control robot
    void MoveRobotPose(orl::Pose);
    void CloseGripper(double width);
    void OpenGripper();
    void Move2InitPostion(int pos);
    bool ReachedTargetPosition(Eigen::Vector3d target);
    orl::Pose PrintCurrState();
};

void handle::Move2InitPostion(int pos) {
    if (pos == 1){
        OpenGripper();
        orl_robot.absolute_cart_motion(target1[0], target1[1], target1[2], execution_time);
        ROS_INFO("Moved to the initial position");
    }
}

void handle::MoveRobotPose(orl::Pose igoal_pose) {
    auto pose_generator = orl::PoseGenerators::MoveToPose(igoal_pose);
    apply_speed_profile(pose_generator, SpeedProfiles::QuinticPolynomialProfile());
    orl_robot.move_cartesian(pose_generator, execution_time);
}

void handle::CloseGripper(double width=0.05) {
    orl_robot.open_gripper(1, width);
//    orl_robot.close_gripper(0.03, 1);
}

void handle::OpenGripper() {
    orl_robot.open_gripper(1, 0.1);
}

orl::Pose handle::PrintCurrState() {
    orl::Pose curr_pos =  orl_robot.get_current_pose(); // ee position
    cout << "position: " << curr_pos.getPosition() << endl;
    cout << "orientation: (" << curr_pos.getOrientation().quaternion.x() << "," << curr_pos.getOrientation().quaternion.y() << "," << curr_pos.getOrientation().quaternion.z() << ")" << endl;
    return curr_pos;
}

bool handle::ReachedTargetPosition(Eigen::Vector3d target) {
    Eigen::Vector3d curr_pos =  orl_robot.get_current_pose().getPosition();
    double dis = sqrt(pow(curr_pos[0]-target[0],2) + pow(curr_pos[1]-target[1],2));
    return dis < 0.01;
}

void handle::Callback(const tengapp::channels &msg) {

    for (int i = 0; i < windows_size-1; ++i) {
        buffer[i] = buffer[i+1];
        buffer1[i] = buffer1[i+1];
        buffer2[i] = buffer[i+1];
    }
    for (int i = 0; i < windows_size*2-1; ++i) {
        buffer3[i] = buffer3[i+1];
    }

    buffer[windows_size-1] = msg.c.at(0);
    buffer1[windows_size-1] = msg.c.at(1);
    buffer2[windows_size-1] = msg.c.at(2);
    buffer3[2*windows_size-1] = msg.c.at(1);
    if(buffer3[0] < 0.01)
        return;

    float m_min = 100, m_min2 = 100;
    float m_max = -100, m_max2 = -100;
    float aver = buffer[windows_size-1];     // average number
    float aver_front = buffer3[windows_size-1];
    float aver_tail = buffer3[2*windows_size-1];
    for (int i = 0; i < windows_size-1; ++i) {
        m_min = min(m_min, buffer[i]);
        m_min2 = min(m_min2, buffer2[i]);

        m_max = max(m_max, buffer[i]);
        m_max2 = max(m_max2, buffer2[i]);

        aver += buffer[i];
        aver_front += buffer3[i];
        aver_tail += buffer3[i+windows_size];
    }
    float max_diff = buffer[windows_size-1] - m_min;
    float max_diff2 = buffer2[windows_size-1] - m_min2;
    float max_diff_descent = m_max - buffer[windows_size-1];

    float aver_diff = aver - m_min;
    aver /= floor(windows_size);
    aver_front /= floor(windows_size);
    aver_tail /= floor(windows_size);
    float aver_diff2 = abs(aver_tail - aver_front);

    // buffer
    if(state == 0){
        cout << buffer[windows_size-1] << " " << m_min << endl;
        cout << "waiting object" << endl;
//        if (max_diff > 1.5 || max_diff_descent > 1.5){
        if (max_diff_descent > 1.5){
            state = 1;
            std_msgs::Int8 msg_int;
            msg_int.data = state;
            pub_state.publish(msg_int);
        }
    }
    orl::Pose curr_pos =  orl_robot.get_current_pose(); // ee position
    cout << curr_pos.getPosition() << endl;
    cout << "max diff: " << max_diff << endl;
    cout << "max diff2: " << max_diff2 << endl;


    if (ReachedTargetPosition(target2)){
        if (state==1){
            state = 2;
            begin_time = ros::Time::now();
            cout << "begin_time: " << begin_time.toSec() << endl;
            cout << "state == 2" << endl;
        }
    }

    if(state == 2){
        end_time = ros::Time::now();
        ros::Duration dura = end_time - begin_time;
        cout << "end_time: " << end_time.toSec() << endl;
        cout << "reached time: " << dura.toSec() << endl;
        cout << "waiting for taking" << endl;
//        if ((max_diff > 1 || max_diff_descent > 1) && dura.toSec() > 2){
        if ((max_diff_descent > 1) && dura.toSec() > 1){
            state = 3;
            std_msgs::Int8 msg_int;
            msg_int.data = state;
            pub_state.publish(msg_int);
            begin_time = ros::Time::now();
        }
    }
	
    if(state==3){
        if (ReachedTargetPosition(target1)){
            end_time = ros::Time::now();
            ros::Duration dura = end_time - begin_time;
            cout << "state: " << state << " " << dura.toSec() << endl;
            if(dura.toSec() > 5)
                state = 0;
        }
    }

    // drop and catch
//    if (state==4){
//        if(max_diff > 0.15 && max_diff < 0.4){
//            cout << "open gripper:" << max_diff << endl;
//            CloseGripper();
//            state = 5;
//            std_msgs::Int8 msg_int;
//            msg_int.data = state;
//            pub_state.publish(msg_int);
//        }
//    }

    //buffer2, close to push

    if(state == 6){
        if (aver_diff2 > 1.5){
            state = 7;
            std_msgs::Int8 msg_int;
            msg_int.data = state;
            pub_state.publish(msg_int);
            ROS_INFO("aver_diff: %f", aver_diff2);
            ilast_pos = orl_robot.get_current_pose().getPosition();
            ilast_pos[1] -= step_dis; // target position
            max_tail = aver_tail;
            begin_time = ros::Time::now();
            curr_cnt = cnt;
        }
        sleep(0.1);
    }


//    if (state == 7 && ReachedTargetPosition(ilast_pos)){
    ros::Duration dur = ros::Time::now() - begin_time;
    double tt = dur.toSec();
    if (state == 7 && (cnt-curr_cnt)>30){
//        cout << "reached target position" << endl;
//        float diff_tail = abs(aver_tail - max_tail);
//        cout << "diff_tail: " << diff_tail << endl;
//        //if (aver_diff2 < 0.1 && aver_diff2 > 0 && diff_tail < 0.3){
//        if (aver_diff2 > 1){
//            state = 7;
//            std_msgs::Int8 msg_int;
//            msg_int.data = state;
//            pub_state.publish(msg_int);
//            ROS_INFO("still moving: %f", aver_diff2);
//            ilast_pos = orl_robot.get_current_pose().getPosition();
//            ilast_pos[1] -= step_dis; // target position
//            max_tail = aver_tail;
//        }
        state = 6;
    }

    cout << "aver_front: " << aver_front << ", aver_tail: " << aver_tail << ", diff: " << aver_diff2 << endl;
    cout << "state: " << state << endl;
    cnt += 1;
}

void handle::CallbackState(const std_msgs::Int8& msg){
    ROS_INFO("state: %d", msg.data);
    if (msg.data == 0){
        cout << "waiting object" << endl;
    }
    if(msg.data == 1){
        sleep(0.5);
        CloseGripper(grasp_width);
        sleep(1);
        orl_robot.absolute_cart_motion(target2[0], target2[1], target2[2], execution_time);
        ROS_INFO("state 0 --> 1, detect object and move to target1");
    }
    if(msg.data == 2){
        cout << "waiting for taking" << endl;
    }
    if(msg.data == 3){
        sleep(0.5);
        OpenGripper();
        sleep(1);
        orl_robot.absolute_cart_motion(target1[0], target1[1], target1[2], execution_time);
        ROS_INFO("state 2 --> 3, detect human touching. Open gripper and move back");
    }
    if (msg.data == 5){
        CloseGripper(grasp_width);
    }

    if (msg.data == 7){
        ROS_INFO("state == 7");
        orl_robot.relative_cart_motion(0, step_dis, 0, execution_time/2);
    }

    if (msg.data == 8){
        ROS_INFO("state == 8");
        orl_robot.relative_cart_motion(0, -step_dis, 0, execution_time/2);
    }
}

void handle::Callback2(const tengapp::channels &msg) {
    // buffer - channel 0
    // buffer2 - channel 1
    for (int i = 0; i < windows_size-1; ++i) {
        buffer[i] = buffer[i+1];
        buffer1[i] = buffer1[i+1];
        buffer2[i] = buffer[i+1];
    }
    for (int i = 0; i < windows_size*2-1; ++i) {
        buffer3[i] = buffer3[i+1];
        buffer4[i] = buffer4[i+1];
        buffer5[i] = buffer5[i+1];
    }

    buffer[windows_size-1] = msg.c.at(1);
    buffer1[windows_size-1] = msg.c.at(0);
    buffer2[windows_size-1] = msg.c.at(2);

    buffer3[2*windows_size-1] = msg.c.at(1);
    buffer4[2*windows_size-1] = msg.c.at(0);
    buffer5[2*windows_size-1] = msg.c.at(2);
    if(buffer3[0] < 0.01) // buffer not fill, return
        return;

    // get the reset voltage
    if (!init_flag){
        int vcnt = 0;
        for (int ii = 0; ii < windows_size*2; ++ii) {
            if (buffer3[ii] > cutoff_max)// max 3, min 1
                vcnt++;
        }
        if (vcnt > windows_size)
            reset_voltage = cutoff_max;
        else
            reset_voltage = cutoff_min;
//        reset_voltage = cutoff_min;
        init_flag = true;
    }

    float buf_max = buffer3[0], buf_min = buffer3[0];
    float buf_max1 = buffer4[0], buf_min1 = buffer4[0];
    float buf_max2 = buffer5[0], buf_min2 = buffer5[0];
    for (int ii = 1; ii < 2*windows_size; ++ii) {
        buf_max = max(buf_max, buffer3[ii]);
        buf_min = min(buf_min, buffer3[ii]);

        buf_max1 = max(buf_max1, buffer4[ii]);
        buf_min1 = min(buf_min1, buffer4[ii]);

        buf_max2 = max(buf_max2, buffer5[ii]);
        buf_min2 = min(buf_min2, buffer5[ii]);
    }
    float max_min_diff = abs(buf_max - buf_min);
    float max_diff1 = buffer4[2*windows_size-1]-buf_max1;
    float min_diff1 = buffer4[2*windows_size-1]-buf_min1;
    float max_diff2 = buffer5[2*windows_size-1]-buf_max2;
    float min_diff2 = buffer5[2*windows_size-1]-buf_min2;

    float buf_aver_front = 0.0, buf_aver_tail = 0.0;
    float buf_aver_front1 = 0.0, buf_aver_tail1 = 0.0;
    float buf_aver_front2 = 0.0, buf_aver_tail2 = 0.0;
//    int max_step = int(windows_size);
//    for (int ii = 0; ii < max_step; ++ii) {
//        buf_aver_tail += buffer[ii];
//        buf_aver_front += buffer[ii+max_step];
//    }
    for (int ii = 0; ii < windows_size; ++ii) {
        buf_aver_tail += buffer3[ii];
        buf_aver_front += buffer3[ii+windows_size];

        buf_aver_tail1 += buffer4[ii];
        buf_aver_front1 += buffer4[ii+windows_size];

        buf_aver_tail2 += buffer5[ii];
        buf_aver_front2 += buffer5[ii+windows_size];
    }
    float aver_tot = (buf_aver_front + buf_aver_tail)/(2.0*windows_size);
    buf_aver_tail /= floor(windows_size);
    buf_aver_front /= floor(windows_size);
    float aver_diff = abs(buf_aver_front - buf_aver_tail);

    float aver_tot1 = (buf_aver_front1 + buf_aver_tail1)/(2.0*windows_size);
    buf_aver_tail1 /= floor(windows_size);
    buf_aver_front1 /= floor(windows_size);
    float aver_diff1 = abs(buf_aver_front1 - buf_aver_tail1);

    float aver_tot2 = (buf_aver_front2 + buf_aver_tail2)/(2.0*windows_size);
    buf_aver_tail2 /= floor(windows_size);
    buf_aver_front2 /= floor(windows_size);
    float aver_diff2 = abs(buf_aver_front2 - buf_aver_tail2);

    ROS_INFO("max_min_diff: %f, aver_diff: %f", max_min_diff, aver_diff);
    ROS_INFO("aver_diff2: %f", aver_diff2);

    /*
     * state = 0, waiting for object
     * state 0 -> 1, detect object, grasp the object, and move to the target position
     * state 2, waiting for taking away the object
     * state 2 -> 3, the object toughed by someone, open the gripper and move back to the initial position
     */
    if (state == 0){
        if (abs(aver_tot-reset_voltage) < 1){
            ROS_INFO("waiting object: aver_tot %f", aver_tot);
            if (max_min_diff > cutoff_width && aver_diff > cutoff_width){
                state = 1;
                std_msgs::Int8 msg_int;
                msg_int.data = state;
                pub_state.publish(msg_int);
            }
        }
    }

    if (ReachedTargetPosition(target2)){
        if (state==1){
            state = 2;
            begin_time = ros::Time::now();
            curr_cnt = cnt;
            ROS_INFO("state == 2");
        }
    }

    if(state == 2){
        end_time = ros::Time::now();
        ros::Duration dura = end_time - begin_time;
//        if ((max_diff > 1 || max_diff_descent > 1) && dura.toSec() > 2){
        if ((max_min_diff > cutoff_width/2) && aver_diff > cutoff_width/2 && cnt-curr_cnt>2*windows_size){
            state = 3;
            std_msgs::Int8 msg_int;
            msg_int.data = state;
            pub_state.publish(msg_int);
            begin_time = ros::Time::now();
            curr_cnt = cnt;
        }
    }

    if(state==3){
        if (ReachedTargetPosition(target1) && cnt-curr_cnt>2*windows_size){
            ROS_INFO("Reached the target position");
            state = 0;
        }
    }

    if (state == 6){
        int tmp = 0;
//        if (reset_voltage > cutoff_width){
//            if (buf_aver_front1 < cutoff_min)
//                tmp = 7;
//            else if(buf_aver_front2 < cutoff_min)
//                tmp = 8;
//        }
//        else{
//            if (buf_aver_front1 > cutoff_max)
//                tmp = 7;
//            else if(buf_aver_front2 > cutoff_max)
//                tmp = 8;
//        }
        // right sensor, falling edge or rising edge
        if (max(abs(max_diff2), abs(min_diff2)) > cutoff_width){
            tmp = 8;
            // rising edge
            if ((buf_aver_front2 > cutoff_max)&&(abs(min_diff2) > cutoff_width)){
                trigger1 = true;
            }
            // falling edge
            else if((buf_aver_front2 < cutoff_min)&&(abs(max_diff2) > cutoff_width)){
                trigger2 = true;
            }
        }
        else if (max(abs(max_diff1), abs(min_diff1)) > cutoff_width){
            tmp = 7;
            if ((buf_aver_front1 > cutoff_max)&&(abs(min_diff1) > cutoff_width)){
                trigger1 = true;
            }
            else if((buf_aver_front1 < cutoff_min)&&(abs(max_diff1) > cutoff_width)){
                trigger2 = true;
            }
        }
        if (tmp > 0 && (trigger1^trigger2)){
            std_msgs::Int8 msg_int;
            msg_int.data = tmp;
            pub_state.publish(msg_int);
            curr_cnt = cnt;
        }
        if (!(trigger1^trigger2))
            state = 7;
    }

    if (state == 7 && cnt-curr_cnt>windows_size){
        trigger1 = false;
        trigger2 = false;
        state = 6;
    }
    cnt++;
    ROS_INFO("State: %d", state);
}
//void handle::CallbackState2(const std_msgs::Int8& msg){
//    ROS_INFO("%d", msg.data);
//}

/*
 *
 * A :(0.4, -0.11, 0.28)
 * B :(0.4, 0.11, 0.28)
 *
 */

void test(){
    handle myhandle = handle();
    const double execution_time = 3;
//    orl::Pose pose = myhandle.GetCurrPose();
//    orl::Pose movep({0,0,0.1}, {0,0,0});
//    myhandle.MoveRobotPose(movep);

//    sleep(2);
//    myhandle.orl_robot.relative_cart_motion(0.0, 0.1, 0.0, execution_time);
//    orl::Pose moveto({0.399672,-0.116832,0.275158}, {0.721533, 0.692175, 0.000451073});
//    myhandle.orl_robot.absolute_cart_motion(myhandle.target1[0], myhandle.target1[1], myhandle.target1[2], execution_time);
//    sleep(3);
//    ros::Time curr = ros::Time::now();
//    myhandle.orl_robot.absolute_cart_motion(myhandle.target2[0], myhandle.target2[1], myhandle.target2[2], execution_time);
//    cout << myhandle.ReachedTargetPosition(myhandle.target2) << endl;
//    ros::Duration dura = ros::Time::now() - curr;
//    cout << dura.toSec() << endl;
//
//    sleep(myhandle.execution_time+1);
//    cout << myhandle.ReachedTargetPosition(myhandle.target2) << endl;
//    myhandle.Move2InitPostion(1);
//    myhandle.orl_robot.relative_cart_motion(0, -0.05, 0, myhandle.execution_time/2);
//    myhandle.orl_robot.relative_cart_motion(0, -0.05, 0, myhandle.execution_time/2);
//    sleep(5);
//    myhandle.PrintCurrState();
//    sleep(3);
//    myhandle.OpenGripper();
//    sleep(2);
//    myhandle.CloseGripper();
//    myhandle.orl_robot.relative_cart_motion(0, -0.05, 0, execution_time);
//    sleep(2);

//    myhandle.Move2InitPostion(1);
//    array<double, 7> curr_joint = myhandle.orl_robot.get_current_Joints();
//    array<double, 7> tar_joint = {{1.05461, -0.181552, -0.482476, -2.37842, -0.120223, 2.18888, -0.076247}};

//    myhandle.orl_robot.joint_motion(tar_joint, 0.05);

//    myhandle.orl_robot.joint_motion(myhandle.target3, 0.05);
//    cout << curr_joint[0] << " " << curr_joint[1] << " " << curr_joint[2] << " " << curr_joint[3] << " " << curr_joint[4] << " " << curr_joint[5] << " " << curr_joint[6] << endl;

    myhandle.CloseGripper(myhandle.grasp_width);
//    sleep(2);
//    myhandle.orl_robot.relative_cart_motion(0, -0.05, 0, execution_time);
//    myhandle.OpenGripper();
//    myhandle.PrintCurrState();
}

int main(int argc, char **argv){
    cout << "node11" << endl;
    ros::init(argc, argv, "tengapp");
//    handle myhandle = handle();
//    myhandle.Move2InitPostion(1);

//    test();
//    ros::spin();

    return 0;
}