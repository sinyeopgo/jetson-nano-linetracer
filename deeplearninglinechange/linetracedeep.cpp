#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#define STDIN_FILENO 0
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include <stdlib.h>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include<iostream>
#include <queue>
#include <iterator>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <ctime>
extern "C" {
#include "dynamixel_sdk.h"
    unsigned int vel_convert(int speed);
    int syncwrite(int port_num, int group_num, int goal_velocity1, int goal_velocity2);
}

using namespace std;
using namespace cv;
using namespace cv::dnn;

// Control table address
#define ADDR_MX_TORQUE_ENABLE           24                  // Control table address is different in Dynamixel model
#define ADDR_MX_GOAL_POSITION           30
#define ADDR_MX_PRESENT_POSITION        36
#define ADDR_MX_MOVING_SPEED            32

// Data Byte Length
#define LEN_MX_GOAL_POSITION            2
#define LEN_MX_PRESENT_POSITION         2
#define LEN_MX_MOVING_SPEED             2

// Protocol version
#define PROTOCOL_VERSION                1.0                 // See which protocol version is used in the Dynamixel

// Default setting
#define DXL1_ID                         1                   // Dynamixel#1 ID: 1
#define DXL2_ID                         2                   // Dynamixel#2 ID: 2
#define BAUDRATE                        2000000
#define DEVICENAME                      "/dev/ttyUSB0"      // Check which port is being used on your controller
                                                            // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE      300                 // Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE      600                // and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD     10                  // Dynamixel moving status threshold

#define ESC_ASCII_VALUE                 0x1b

void secondline(Mat roi1, Mat roi2);
int getch()
{
#if defined(__linux__) || defined(__APPLE__)
    struct termios oldt, newt;
    int ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
#elif defined(_WIN32) || defined(_WIN64)
    return _getch();
#endif
}

int kbhit(void)
{
#if defined(__linux__) || defined(__APPLE__)
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if (ch != EOF)
    {
        ungetc(ch, stdin);
        return 1;
    }

    return 0;
#elif defined(_WIN32) || defined(_WIN64)
    return _kbhit();
#endif
}

constexpr float CONFIDENCE_THRESHOLD = 0.9;
constexpr float NMS_THRESHOLD = 0.4;
constexpr int NUM_CLASSES = 1;

// colors for bounding boxes
const cv::Scalar colors[] = {
    {0, 255, 255},
    {255, 255, 0},
    {0, 255, 0},
    {255, 0, 0}
};
const auto NUM_COLORS = sizeof(colors) / sizeof(colors[0]);

string codec1 = "nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=(int)640, height=(int)360, format=(string)NV12, framerate=(fraction)15/1 ! \
     nvvidconv flip-method=0 ! video/x-raw, width=(int)640, height=(int)360, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";

Point2d rcpt[2];
int main()
{
    std::vector<std::string> class_names;
    {
        std::ifstream class_file("robot.names");
        if (!class_file)
        {
            std::cerr << "failed to open classes.txt\n";
            return 0;
        }

        std::string line;
        while (std::getline(class_file, line))
            class_names.push_back(line);
    }

    auto net = cv::dnn::readNetFromDarknet("yolov4-tiny-robot-288.cfg", "yolov4-tiny-robot-288_final.weights");
    net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
    net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
    /*net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);*/
    auto output_names = net.getUnconnectedOutLayersNames();

    //VIdeo on
    VideoCapture cap1(codec1, CAP_GSTREAMER);
    if (!cap1.isOpened()) { cout << "Video error" << endl; }

    float videoFPS = cap1.get(cv::CAP_PROP_FPS);
    int videoWidth = cap1.get(cv::CAP_PROP_FRAME_WIDTH);
    int videoHeight = cap1.get(cv::CAP_PROP_FRAME_HEIGHT);

    VideoWriter  vstream;

    vstream.open("streaming.avi", cv::VideoWriter::fourcc('A', 'V', 'C', '1'), videoFPS, cv::Size(videoWidth , videoHeight ), true);

    // Initialize PortHandler Structs
    // Set the port path
    // Get methods and members of PortHandlerLinux or PortHandlerWindows
    int port_num = portHandler(DEVICENAME);

    // Initialize PacketHandler Structs
    packetHandler();

    // Initialize Groupsyncwrite instance
    //int group_num = groupSyncWrite(port_num, PROTOCOL_VERSION, ADDR_MX_GOAL_POSITION, LEN_MX_GOAL_POSITION);
    int group_num = groupSyncWrite(port_num, PROTOCOL_VERSION, ADDR_MX_MOVING_SPEED, LEN_MX_GOAL_POSITION);

    //int index = 0;
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    //uint8_t dxl_addparam_result = false;            // AddParam result
    //int dxl_goal_position[2] = { DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE };  // Goal position

    uint8_t dxl_error = 0;                          // Dynamixel error
    //uint16_t dxl1_present_position = 0, dxl2_present_position = 0;                          // Present position

    // Open port
    if (openPort(port_num))
    {
        printf("Succeeded to open the port!\n");
    }
    else
    {
        printf("Failed to open the port!\n");
        printf("Press any key to terminate...\n");
        getch();
        return 0;
    }

    // Set port baudrate
    if (setBaudRate(port_num, BAUDRATE))
    {
        printf("Succeeded to change the baudrate!\n");
    }
    else
    {
        printf("Failed to change the baudrate!\n");
        printf("Press any key to terminate...\n");
        getch();
        return 0;
    }

    // Enable Dynamixel#1 Torque
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE);
    if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
    {
        printf("%s\n", getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
    }
    else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
    {
        printf("%s\n", getRxPacketError(PROTOCOL_VERSION, dxl_error));
    }
    else
    {
        printf("Dynamixel#%d has been successfully connected \n", DXL1_ID);
    }

    // Enable Dynamixel#2 Torque
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL2_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE);
    if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
    {
        printf("%s\n", getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
    }
    else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
    {
        printf("%s\n", getRxPacketError(PROTOCOL_VERSION, dxl_error));
    }
    else
    {
        printf("Dynamixel#%d has been successfully connected \n", DXL2_ID);
    }

    Mat frame,gray, dst, roi1, roi2,blob;
    Point2d prevpt(320, 60);
    Point2d cpt[2];
    int minlb;
    int thres;
    double ptdistance;
    vector<double> mindistance;
    std::vector<cv::Mat> detections;
    double error;
    double myproms;
    int cnt;
    bool strdet = false;
    int mode = 0;
    int laneflags = 0;
    int initspeed = 300;
    bool rc = false;
    while (1)
    {
        int64 t1 = getTickCount();

        cap1 >> frame;
        if (frame.empty())
        {
            cv::waitKey();
            return -1;
            break;
        }

        cv::dnn::blobFromImage(frame, blob, 0.00392, cv::Size(288, 288), cv::Scalar(), true, false, CV_32F);
        net.setInput(blob);

        net.forward(detections, output_names);

        std::vector<int> indices[NUM_CLASSES];
        std::vector<cv::Rect> boxes[NUM_CLASSES];
        std::vector<float> scores[NUM_CLASSES];

        for (auto& output : detections)
        {
            cout<<"good"<<endl;
            const auto num_boxes = output.rows;
            for (int i = 0; i < num_boxes; i++)
            {
                auto x = output.at<float>(i, 0) * frame.cols;
                auto y = output.at<float>(i, 1) * frame.rows;
                auto width = output.at<float>(i, 2) * frame.cols;
                auto height = output.at<float>(i, 3) * frame.rows;
                cv::Rect rect(x - width / 2, y - height / 2, width, height);

                for (int c = 0; c < NUM_CLASSES; c++)
                {
                    auto confidence = *output.ptr<float>(i, 5 + c);
                    if (confidence >= CONFIDENCE_THRESHOLD)
                    {
                        boxes[c].push_back(rect);
                        scores[c].push_back(confidence);
                    }
                }
            }
        }

        for (int c = 0; c < NUM_CLASSES; c++)
            cv::dnn::NMSBoxes(boxes[c], scores[c], 0.7, NMS_THRESHOLD, indices[c]);

        for (int c = 0; c < NUM_CLASSES; c++)
        {
            for (size_t i = 0; i < indices[c].size(); ++i)
            {
                auto idx = indices[c][i];
                const auto& rect = boxes[c][idx];
                int rectarea = rect.size().area();
                if(rectarea > 20000 && rectarea < 50000){
                    initspeed = 300;
                    initspeed -= rectarea/250;
                }
                else initspeed = 300;
                if (((rect.x + rect.width / 2) > 160) && ((rect.x + rect.width / 2) < 360) && initspeed <= 200) rc = true;
                else rc = false;
                // cout << "rect center : " << rect.x + rect.width / 2 << endl;
                // cout << "rect area : " << rectarea << endl;
            }
            if (indices[c].size()<1) initspeed = 300;
        }

        cvtColor(frame, gray, COLOR_BGR2GRAY);
        gray = gray + 100 - mean(gray)[0];
        thres = 180;
        threshold(gray, gray, thres, 255, THRESH_BINARY);

        dst = gray(Rect(0, gray.rows / 3 * 2, gray.cols, gray.rows / 3));
        roi1 = dst(Rect(0, 0, dst.cols / 3, dst.rows));
        roi2 = dst(Rect(dst.cols / 3 * 2, 0, dst.cols / 3, dst.rows ));

        Mat labels, stats, centroids;
        cnt = connectedComponentsWithStats(dst, labels, stats, centroids);
        //linetrace
        if (cnt > 1) {
            for (int i = 1; i < cnt; i++) {
                double* p2 = centroids.ptr<double>(i);
                ptdistance = abs(p2[0] - prevpt.x);
                mindistance.push_back(ptdistance);
            }
            minlb = min_element(mindistance.begin(), mindistance.end()) - mindistance.begin();
            cpt[0] = Point2d(centroids.at<double>(minlb + 1, 0), centroids.at<double>(minlb + 1, 1));
            
            if (abs(prevpt.x - cpt[0].x) > 50&&dst.at<uchar>(cpt[0].y,cpt[0].x) != 255)
                cpt[0] = prevpt;
            mindistance.clear();
        }
        else cpt[0] = prevpt;

        prevpt = cpt[0];
        cvtColor(dst, dst, COLOR_GRAY2BGR);

        cpt[0].y += frame.rows / 3 * 2;
        circle(frame, cpt[0], 4, Scalar(0, 0, 255), -1);

        //error
        error = dst.cols / 2 - cpt[0].x;
        // cout << "error: " << error << endl;

        secondline(roi1, roi2);
        //curve or not
        if (abs(error) < 100&&rc == true) {
            strdet = true;
            rc = false;
        }
        else {
            strdet = false;
        }
        if (strdet == true) {
            if (rcpt[0].x != 10000 && rcpt[1].x == 10000) {
                // cout<<"******************change******************"<<endl;
                prevpt = rcpt[0];
                laneflags = 1;
                strdet = false;
            }
            else if (rcpt[0].x == 10000 && rcpt[1].x != 10000) {
                // cout<<"******************change******************"<<endl;
                rcpt[1].x += 480;
                prevpt = rcpt[1];
                laneflags = 1;
                strdet = false;

            }
            else {
                // cout << "no line" << endl;
                laneflags = 0;
            }
        }
        else
            laneflags = 0;
        
        switch (mode)
        {
        case 0:
            mode = 0;
            if (laneflags == 1) mode = 1;
            break;
        case 1:
            mode = 1;
            if (abs(error) < 50) mode = 0;
            break;
        default:
            break;
        }
        int L_speed;
        int R_speed;
        if (mode == 1) {
            L_speed = 250 - error * 0.2;
            R_speed = 250 + error * 0.2;
        }
        else {
            L_speed = initspeed - error * 0.5;
            R_speed = initspeed + error * 0.5;
        }
        syncwrite(port_num, group_num, L_speed, -R_speed);
        for (int c = 0; c < NUM_CLASSES; c++)
        {
            for (size_t i = 0; i < indices[c].size(); ++i)
            {
                const auto color = colors[c % NUM_COLORS];

                auto idx = indices[c][i];
                const auto& rect = boxes[c][idx];
                cv::rectangle(frame, cv::Point(rect.x, rect.y), cv::Point(rect.x + rect.width, rect.y + rect.height), color, 3);

                std::ostringstream label_ss;
                label_ss << class_names[c] << ": " << std::fixed << std::setprecision(2) << scores[c][idx];
                auto label = label_ss.str();

                int baseline;
                auto label_bg_sz = cv::getTextSize(label.c_str(), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, 1, &baseline);
                cv::rectangle(frame, cv::Point(rect.x, rect.y - label_bg_sz.height - baseline - 10), cv::Point(rect.x + label_bg_sz.width, rect.y), color, cv::FILLED);
                cv::putText(frame, label.c_str(), cv::Point(rect.x, rect.y - baseline - 5), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(0, 0, 0));
            }
        }
        vstream << frame;
        //printf("speed1:%d,speed2:%d\n", L_speed, -R_speed);
        // imshow("video1", frame);
        // imshow("video2", dst);
        if (kbhit() == 1) break;
        // waitKey(2);
        int64 t2 = getTickCount();
        myproms = (t2 - t1) * 1000 / getTickFrequency();
        cout << "time: " << myproms << endl;
    }
    syncwrite(port_num, group_num, 0, 0);
    // Disable Dynamixel#1 Torque
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE);
    if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
    {
        printf("%s\n", getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
    }
    else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
    {
        printf("%s\n", getRxPacketError(PROTOCOL_VERSION, dxl_error));
    }

    // Disable Dynamixel#2 Torque
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL2_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE);
    if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
    {
        printf("%s\n", getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
    }
    else if ((dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION)) != 0)
    {
        printf("%s\n", getRxPacketError(PROTOCOL_VERSION, dxl_error));
    }

    // Close port
    closePort(port_num);

    return 0;
}

//-1023~1023 -> +(CCW) 0~1023, -(CW) 1024~2047
unsigned int vel_convert(int speed)
{
    unsigned int temp;
    if (speed > 1023) speed = 1023;
    else if (speed < -1023) speed = -1023;

    if (speed >= 0) temp = (unsigned int)speed;
    else temp = (unsigned int)(-speed + 1023);

    return temp;
}

int syncwrite(int port_num, int group_num, int goal_velocity1, int goal_velocity2)
{
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    uint8_t dxl_addparam_result = false;            // AddParam result
     // Add Dynamixel#1 goal position value to the Syncwrite storage
    dxl_addparam_result = groupSyncWriteAddParam(group_num, DXL1_ID, vel_convert(goal_velocity1), LEN_MX_GOAL_POSITION);
    if (dxl_addparam_result != true)
    {
        fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL1_ID);
        return 0;
    }

    // Add Dynamixel#2 goal position value to the Syncwrite parameter storage
    dxl_addparam_result = groupSyncWriteAddParam(group_num, DXL2_ID, vel_convert(goal_velocity2), LEN_MX_GOAL_POSITION);
    if (dxl_addparam_result != true)
    {
        fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", DXL2_ID);
        return 0;
    }

    // Syncwrite goal position
    groupSyncWriteTxPacket(group_num);
    if ((dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION)) != COMM_SUCCESS)
        printf("%s\n", getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));

    // Clear syncwrite parameter storage
    groupSyncWriteClearParam(group_num);
    return 0;

}
void secondline(Mat roi1,Mat roi2) {
    Mat roi[2];
    Point2d cpt[2];
    int minlb1, minlb2;
    double ptdistance1, ptdistance2;
    vector<double> mindistance1, mindistance2;
    int cnt2, cnt3;
    Mat labels1, labels2, stats1, stats2, centroids1, centroids2;
    cnt2 = connectedComponentsWithStats(roi1, labels1, stats1, centroids1);
    cnt3 = connectedComponentsWithStats(roi2, labels2, stats2, centroids2);
    //left
    if (cnt2 > 1) {
        for (int i = 1; i < cnt2; i++) {
            int* p1 = stats1.ptr<int>(i);
            ptdistance1 = abs(p1[4]);
            mindistance1.push_back(ptdistance1);
        }
        minlb1 = max_element(mindistance1.begin(), mindistance1.end()) - mindistance1.begin();
        if (stats1.ptr<int>(minlb1 + 1)[4] < 20)
            cpt[0] = Point(10000, 10000);
        else
            cpt[0] = Point2d(centroids1.at<double>(minlb1 + 1, 0), centroids1.at<double>(minlb1 + 1, 1));
        mindistance1.clear();
    }
    else cpt[0] = Point(10000, 10000);

    //right
    if (cnt3 > 1) {
        for (int i = 1; i < cnt3; i++) {
            int* p1 = stats2.ptr<int>(i);
            ptdistance2 = abs(p1[4]);
            mindistance2.push_back(ptdistance2);
        }
        minlb2 = max_element(mindistance2.begin(), mindistance2.end()) - mindistance2.begin();
        if (stats2.ptr<int>(minlb2 + 1)[4] < 20)
            cpt[1] = Point(10000, 10000);
        else
            cpt[1] = Point2d(centroids2.at<double>(minlb2 + 1, 0), centroids2.at<double>(minlb2 + 1, 1));
        mindistance2.clear();
    }
    else cpt[1] = Point(10000, 10000);
    

    cvtColor(roi1, roi1, COLOR_GRAY2BGR);
    cvtColor(roi2, roi2, COLOR_GRAY2BGR);

    //circle(roi1, cpt[0], 2, Scalar(0, 0, 255), 2);
    //circle(roi2, cpt[1], 2, Scalar(0, 0, 255), 2);

    rcpt[0] = cpt[0];
    rcpt[1] = cpt[1];

}