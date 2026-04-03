/**
* NAO Robot SLAM Bridge - Monocular-Inertial with IMU
*
* Connects to NAO robot, receives MJPEG video stream and IMU data
* and feeds them to ORB-SLAM3 for visual-inertial odometry
*
* Usage: ./mono_inertial_nao path_to_vocabulary path_to_settings [robot_ip]
*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<thread>
#include<mutex>
#include<deque>
#include<opencv2/core/core.hpp>
#include<opencv2/imgproc.hpp>
#include<opencv2/highgui.hpp>
#include<System.h>
#include "ImuTypes.h"

using namespace std;

// Thread-safe buffer for IMU measurements
class ImuBuffer {
public:
    void push(double timestamp, cv::Point3f acc, cv::Point3f gyro) {
        lock_guard<mutex> lock(mtx);
        imu_data.push_back({timestamp, acc, gyro});
    }

    // Get all IMU measurements up to the given timestamp
    vector<ORB_SLAM3::IMU::Point> getUpTo(double timestamp) {
        lock_guard<mutex> lock(mtx);
        vector<ORB_SLAM3::IMU::Point> result;

        while (!imu_data.empty() && imu_data.front().timestamp <= timestamp) {
            auto& data = imu_data.front();
            result.push_back(ORB_SLAM3::IMU::Point(
                data.acc.x, data.acc.y, data.acc.z,
                data.gyro.x, data.gyro.y, data.gyro.z,
                data.timestamp
            ));
            imu_data.pop_front();
        }

        return result;
    }

private:
    struct ImuData {
        double timestamp;
        cv::Point3f acc;
        cv::Point3f gyro;
    };

    mutex mtx;
    deque<ImuData> imu_data;
};

// Simple NAO IMU reader via SSH/command
class NaoImuReader {
public:
    NaoImuReader(const string& ip, ImuBuffer& buf) : robot_ip(ip), buffer(buf), running(false) {}

    void start() {
        running = true;
        thread = std::thread(&NaoImuReader::readLoop, this);
    }

    void stop() {
        running = false;
        if (thread.joinable()) {
            thread.join();
        }
    }

private:
    void readLoop() {
        // Use ssh to continuously read IMU data from NAO
        // This is a simplified approach - for production, use qicli or NAOqi API

        string cmd = "sshpass -p 'nao' ssh -o StrictHostKeyChecking=no nao@" + robot_ip +
                     " 'while true; do qicli call --method getValue \"Device/SubDeviceList/InertialSensor/Accelerometer/X/Sensor/Value\" "
                     "qicli call --method getValue \"Device/SubDeviceList/InertialSensor/Accelerometer/Y/Sensor/Value\" "
                     "qicli call --method getValue \"Device/SubDeviceList/InertialSensor/Accelerometer/Z/Sensor/Value\" "
                     "qicli call --method getValue \"Device/SubDeviceList/InertialSensor/Gyroscope/X/Sensor/Value\" "
                     "qicli call --method getValue \"Device/SubDeviceList/InertialSensor/Gyroscope/Y/Sensor/Value\" "
                     "qicli call --method getValue \"Device/SubDeviceList/InertialSensor/Gyroscope/Z/Sensor/Value\" "
                     "2>/dev/null; sleep 0.01; done'";

        FILE* pipe = popen(cmd.c_str(), "r");
        if (!pipe) {
            cerr << "Failed to open IMU pipe" << endl;
            return;
        }

        char line[256];
        chrono::steady_clock::time_point start_time = chrono::steady_clock::now();

        while (running && fgets(line, sizeof(line), pipe)) {
            // Parse the output (format depends on qicli output)
            float acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z;
            if (sscanf(line, "%f %f %f %f %f %f", &acc_x, &acc_y, &acc_z, &gyro_x, &gyro_y, &gyro_z) == 6) {
                // Calculate timestamp
                double timestamp = chrono::duration_cast<chrono::duration<double>>(
                    chrono::steady_clock::now() - start_time).count();

                // NAO IMU: acceleration in m/s^2 (need to convert from g), gyro in deg/s (need rad/s)
                cv::Point3f acc(acc_x * 9.81, acc_y * 9.81, acc_z * 9.81);
                cv::Point3f gyro(gyro_x * M_PI/180.0, gyro_y * M_PI/180.0, gyro_z * M_PI/180.0);

                buffer.push(timestamp, acc, gyro);
            }
        }

        pclose(pipe);
    }

    string robot_ip;
    ImuBuffer& buffer;
    atomic<bool> running;
    std::thread thread;
};

int main(int argc, char **argv)
{
    if(argc < 3)
    {
        cerr << endl << "Usage: ./mono_inertial_nao path_to_vocabulary path_to_settings [robot_ip]" << endl;
        cerr << endl << "Example: ./mono_inertial_nao ../../Vocabulary/ORBvoc.txt NAO_640x480.yaml" << endl;
        return 1;
    }

    string robot_ip = (argc >= 4) ? string(argv[3]) : "169.254.81.31";
    string stream_url = "http://" + robot_ip + ":8080/stream";

    cout << endl << "-------" << endl;
    cout << "NAO SLAM Bridge - Monocular-Inertial" << endl;
    cout << "Robot IP: " << robot_ip << endl;
    cout << "Stream URL: " << stream_url << endl;
    cout << "Vocabulary: " << argv[1] << endl;
    cout << "Settings: " << argv[2] << endl;
    cout << "-------" << endl << endl;

    // IMU buffer
    ImuBuffer imu_buffer;

    // Start IMU reader
    cout << "Starting IMU reader..." << endl;
    NaoImuReader imu_reader(robot_ip, imu_buffer);
    imu_reader.start();

    // Give IMU reader time to collect some data
    this_thread::sleep_for(chrono::milliseconds(500));

    // Open video stream
    cout << "Connecting to camera stream..." << endl;
    cv::VideoCapture cap(stream_url);
    if(!cap.isOpened())
    {
        cerr << "Failed to connect to stream: " << stream_url << endl;
        cerr << "Make sure robot_streamer.py is running on the NAO" << endl;
        imu_reader.stop();
        return 1;
    }

    cout << "Connected! Starting SLAM..." << endl;

    // Create SLAM system with IMU
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::IMU_MONOCULAR, true);
    float imageScale = SLAM.GetImageScale();

    // Main loop
    cv::Mat im, frame_bgr;
    double timestamp = 0.0;
    int frame_count = 0;

    cout << "Press ESC to stop and save trajectory" << endl << endl;

    chrono::steady_clock::time_point start_time = chrono::steady_clock::now();

    while(true)
    {
        // Read frame from stream
        cap >> frame_bgr;

        if(frame_bgr.empty())
        {
            cerr << "Failed to read frame from stream" << endl;
            break;
        }

        // Convert BGR to RGB (NAO sends RGB)
        cv::cvtColor(frame_bgr, im, cv::COLOR_BGR2RGB);

        // Calculate timestamp (seconds since start)
        timestamp = chrono::duration_cast<chrono::duration<double>>(
            chrono::steady_clock::now() - start_time).count();

        // Resize if needed
        if(imageScale != 1.f)
        {
            int width = im.cols * imageScale;
            int height = im.rows * imageScale;
            cv::resize(im, im, cv::Size(width, height));
        }

        // Get IMU measurements up to current timestamp
        vector<ORB_SLAM3::IMU::Point> vImuMeas = imu_buffer.getUpTo(timestamp);

        chrono::steady_clock::time_point t1 = chrono::steady_clock::now();

        // Pass image and IMU data to SLAM system
        SLAM.TrackMonocular(im, timestamp, vImuMeas);

        chrono::steady_clock::time_point t2 = chrono::steady_clock::now();

        double ttrack = chrono::duration_cast<chrono::duration<double>>(t2 - t1).count();

        // Display frame with tracking info
        frame_count++;
        if(frame_count % 30 == 0)
        {
            cout << "Frame: " << frame_count << " | Track time: " << (ttrack*1000) << "ms | IMU samples: " << vImuMeas.size() << endl;
        }

        // Show frame
        cv::imshow("NAO SLAM - Inertial", frame_bgr);

        // ESC to quit
        if(cv::waitKey(1) == 27)
        {
            cout << endl << "ESC pressed - shutting down..." << endl;
            break;
        }
    }

    // Stop all threads
    SLAM.Shutdown();
    imu_reader.stop();

    // Save camera trajectory
    cout << "Saving trajectory to KeyFrameTrajectory.txt" << endl;
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_Inertial.txt");
    SLAM.SaveTrajectoryTUM("FrameTrajectory_Inertial.txt");

    cout << "Done!" << endl;
    return 0;
}
