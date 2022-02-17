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
#include "opencv2/opencv.hpp"
#include<iostream>
extern "C" {
#include "dynamixel_sdk.h"

    unsigned int vel_convert(int speed);
    int syncwrite(int port_num, int group_num, int goal_velocity1, int goal_velocity2);
}

using namespace std;
using namespace cv;

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

string codec1 = "nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM), width=(int)640, height=(int)360, format=(string)NV12, framerate=(fraction)15/1 ! \
     nvvidconv flip-method=0 ! video/x-raw, width=(int)640, height=(int)360, format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";


int main()
{
    //VIdeo on
    VideoCapture cap1(codec1, CAP_GSTREAMER);
    if (!cap1.isOpened()) { cout << "Video error" << endl; }

    // Initialize PortHandler Structs
    // Set the port path
    // Get methods and members of PortHandlerLinux or PortHandlerWindows
    int port_num = portHandler(DEVICENAME);

    // Initialize PacketHandler Structs
    packetHandler();

    // Initialize Groupsyncwrite instance
    //int group_num = groupSyncWrite(port_num, PROTOCOL_VERSION, ADDR_MX_GOAL_POSITION, LEN_MX_GOAL_POSITION);
    int group_num = groupSyncWrite(port_num, PROTOCOL_VERSION, ADDR_MX_MOVING_SPEED, LEN_MX_GOAL_POSITION);

    int index = 0;
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    uint8_t dxl_addparam_result = false;            // AddParam result
    int dxl_goal_position[2] = { DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE };  // Goal position

    uint8_t dxl_error = 0;                          // Dynamixel error
    uint16_t dxl1_present_position = 0, dxl2_present_position = 0;                          // Present position

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

    Mat frame1, gray, dst;
    Point2d prevpt1(110, 60);
    Point2d prevpt2(520, 60);
    Point2d cpt[2];
    Point2d fpt;
    int minlb[2];
    int thres;
    double ptdistance[2];
    double threshdistance[2];
    vector<double> mindistance1;
    vector<double> mindistance2;
    double error;
    double myproms;
    int count = 0;
    while (1)
    {
        int64 t1 = getTickCount();

        cap1 >> frame1;

        cvtColor(frame1, gray, COLOR_BGR2GRAY);
        cout << "mean: " << mean(gray)[0] << endl;
        gray = gray + 100 - mean(gray)[0];
        cout << "mean2: " << mean(gray)[0] << endl;
        thres = 160;
        threshold(gray, gray, thres, 255, THRESH_BINARY);

        dst = gray(Rect(0, gray.rows / 3 * 2, gray.cols, gray.rows / 3));

        Mat labels, stats, centroids;
        int cnt = connectedComponentsWithStats(dst, labels, stats, centroids);
        if (cnt > 1) {
            for (int i = 1; i < cnt; i++) {
                double* p = centroids.ptr<double>(i);
                ptdistance[0] = abs(p[0] - prevpt1.x);
                ptdistance[1] = abs(p[0] - prevpt2.x);
                mindistance1.push_back(ptdistance[0]);
                mindistance2.push_back(ptdistance[1]);
            }
            for (int i : mindistance1) cout << i << " ";
            cout << endl;
            for (int i : mindistance2) cout << i << " ";
            cout << endl;

            threshdistance[0] = *min_element(mindistance1.begin(), mindistance1.end());
            threshdistance[1] = *min_element(mindistance2.begin(), mindistance2.end());

            minlb[0] = min_element(mindistance1.begin(), mindistance1.end()) - mindistance1.begin();
            minlb[1] = min_element(mindistance2.begin(), mindistance2.end()) - mindistance2.begin();

            cpt[0] = Point2d(centroids.at<double>(minlb[0] + 1, 0), centroids.at<double>(minlb[0] + 1, 1));
            cpt[1] = Point2d(centroids.at<double>(minlb[1] + 1, 0), centroids.at<double>(minlb[1] + 1, 1));

            if (threshdistance[0] > 100) cpt[0] = prevpt1;
            if (threshdistance[1] > 100) cpt[1] = prevpt2;

            mindistance1.clear();
            mindistance2.clear();
        }
        else {
            cpt[0] = prevpt1;
            cpt[1] = prevpt2;
        }

        prevpt1 = cpt[0];
        prevpt2 = cpt[1];

        fpt.x = (cpt[0].x + cpt[1].x) / 2;
        fpt.y = (cpt[0].y + cpt[1].y) / 2 + gray.rows / 3 * 2;
        cvtColor(dst, dst, COLOR_GRAY2BGR);

        circle(frame1, fpt, 2, Scalar(0, 0, 255), 2);
        circle(dst, cpt[0], 2, Scalar(0, 0, 255), 2);
        circle(dst, cpt[1], 2, Scalar(255, 0, 0), 2);

        error = dst.cols / 2 - fpt.x;

        int L_speed = 300 - error / 7 * 5;
        int R_speed = 300 + error / 7 * 5;

        cout << "error: " << error << endl;

        syncwrite(port_num, group_num, L_speed, -R_speed);
        printf("speed1:%d,speed2:%d\n", L_speed, -R_speed);
        usleep(10000);

        imshow("video1", frame1);
        /*imshow("video2", dst);*/
        int64 t2 = getTickCount();
        myproms = (t2 - t1) * 1000 / getTickFrequency();
        cout << "time: " << myproms << endl;
        if (waitKey(33) == 27) break;
    }

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
