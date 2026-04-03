/**
* NAO Robot SLAM Bridge - With Robot Control
*
* Connects to NAO robot, receives MJPEG video stream
* and feeds to ORB-SLAM3 while controlling robot with keyboard
*
* Usage: ./mono_nao_control path_to_vocabulary path_to_settings [robot_ip]
*
* Controls:
*   W/S: forward/backward
*   A/D: strafe left/right
*   Q/E: turn left/right
*   SPACE: stop
*   ESC: quit
*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<thread>
#include<mutex>
#include<deque>
#include<map>
#include<opencv2/core/core.hpp>
#include<opencv2/imgproc.hpp>
#include<opencv2/highgui.hpp>
#include<System.h>

using namespace std;

// Simple TCP client to send commands to NAO
class NAOController {
public:
    NAOController(const string& ip) : robot_ip(ip), connected(false) {
        // Connect to NAOqi via qicli
        // For simplicity, we use SSH commands
        string cmd = "sshpass -p 'nao' ssh -o StrictHostKeyChecking=no nao@" + ip +
                     " 'qicli call ALMotion.setWalkTargetVelocity'";

        // We'll build the command dynamically for each movement
        cout << "NAO Controller initialized for " << ip << endl;
    }

    void setVelocity(float x, float y, float theta) {
        // Send velocity command via SSH/qicli
        char cmd[512];
        snprintf(cmd, sizeof(cmd),
            "sshpass -p 'nao' ssh -o StrictHostKeyChecking=no nao@%s "
            "'qicli call ALMotion.setWalkTargetVelocity FLOAT:%f FLOAT:%f FLOAT:%f FLOAT:%f' 2>/dev/null &",
            robot_ip.c_str(), x, y, theta, 0.0f);
        system(cmd);
    }

    void wakeUp() {
        string cmd = "sshpass -p 'nao' ssh -o StrictHostKeyChecking=no nao@" + robot_ip +
                     " 'qicli call ALMotion.wakeUp' 2>/dev/null &";
        system(cmd.c_str());
    }

private:
    string robot_ip;
    bool connected;
};


// Keyboard state tracker
class KeyState {
public:
    KeyState() : last_update(chrono::steady_clock::now()) {}

    void press(char key) {
        lock_guard<mutex> lock(mtx);
        pressed_keys[key] = chrono::steady_clock::now();
    }

    void release(char key) {
        lock_guard<mutex> lock(mtx);
        pressed_keys.erase(key);
    }

    // Get currently pressed movement keys (auto-releases after 100ms)
    map<char, bool> getMovementKeys() {
        lock_guard<mutex> lock(mtx);
        auto now = chrono::steady_clock::now();

        // Remove stale keys
        for (auto it = pressed_keys.begin(); it != pressed_keys.end(); ) {
            auto age = chrono::duration_cast<chrono::milliseconds>(now - it->second).count();
            if (age > 150) {
                it = pressed_keys.erase(it);
            } else {
                ++it;
            }
        }

        map<char, bool> result;
        for (auto& kv : pressed_keys) {
            result[kv.first] = true;
        }
        return result;
    }

    void clear() {
        lock_guard<mutex> lock(mtx);
        pressed_keys.clear();
    }

private:
    mutex mtx;
    map<char, chrono::steady_clock::time_point> pressed_keys;
    chrono::steady_clock::time_point last_update;
};


int main(int argc, char **argv)
{
    if(argc < 3)
    {
        cerr << endl << "Usage: ./mono_nao_control path_to_vocabulary path_to_settings [robot_ip]" << endl;
        return 1;
    }

    string robot_ip = (argc >= 4) ? string(argv[3]) : "169.254.81.31";
    string stream_url = "http://" + robot_ip + ":8080/stream";

    cout << endl << "-------" << endl;
    cout << "NAO SLAM Bridge - With Robot Control" << endl;
    cout << "Robot IP: " << robot_ip << endl;
    cout << "Stream URL: " << stream_url << endl;
    cout << "-------" << endl << endl;

    // Initialize NAO controller
    NAOController nao(robot_ip);
    nao.wakeUp();

    KeyState keys;

    // Open video stream
    cout << "Connecting to camera stream..." << endl;
    cv::VideoCapture cap(stream_url);
    if(!cap.isOpened())
    {
        cerr << "Failed to connect to stream: " << stream_url << endl;
        return 1;
    }

    cout << "Connected! Starting SLAM..." << endl;

    // Create SLAM system
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, true);
    float imageScale = SLAM.GetImageScale();

    cout << endl << "Controls:" << endl;
    cout << "  W/S: forward/backward" << endl;
    cout << "  A/D: strafe left/right" << endl;
    cout << "  Q/E: turn left/right" << endl;
    cout << "  SPACE: stop" << endl;
    cout << "  ESC: quit" << endl << endl;

    // Main loop
    cv::Mat im, frame_bgr;
    double timestamp = 0.0;
    int frame_count = 0;

    chrono::steady_clock::time_point start_time = chrono::steady_clock::now();
    chrono::steady_clock::time_point last_cmd = start_time;

    float cmd_x = 0, cmd_y = 0, cmd_theta = 0;

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

        // Calculate timestamp
        timestamp = chrono::duration_cast<chrono::duration<double>>(
            chrono::steady_clock::now() - start_time).count();

        // Resize if needed
        if(imageScale != 1.f)
        {
            int width = im.cols * imageScale;
            int height = im.rows * imageScale;
            cv::resize(im, im, cv::Size(width, height));
        }

        chrono::steady_clock::time_point t1 = chrono::steady_clock::now();

        // Pass image to SLAM system
        SLAM.TrackMonocular(im, timestamp);

        chrono::steady_clock::time_point t2 = chrono::steady_clock::now();

        double ttrack = chrono::duration_cast<chrono::duration<double>>(t2 - t1).count();

        // Process keyboard input
        int key = cv::waitKey(1) & 0xFF;
        if (key != 255) {
            if (key == 27) {  // ESC
                break;
            } else if (key == 32) {  // SPACE - stop
                cmd_x = cmd_y = cmd_theta = 0;
                keys.clear();
            } else {
                keys.press(tolower(key));
            }
        }

        // Update movement commands every 100ms
        auto now = chrono::steady_clock::now();
        if (chrono::duration_cast<chrono::milliseconds>(now - last_cmd).count() > 100) {
            auto active_keys = keys.getMovementKeys();

            cmd_x = cmd_y = cmd_theta = 0;
            if (active_keys.count('w')) cmd_x = 0.5f;
            else if (active_keys.count('s')) cmd_x = -0.5f;
            if (active_keys.count('a')) cmd_y = 0.5f;
            else if (active_keys.count('d')) cmd_y = -0.5f;
            if (active_keys.count('q')) cmd_theta = 0.5f;
            else if (active_keys.count('e')) cmd_theta = -0.5f;

            // Send command if any movement
            if (cmd_x != 0 || cmd_y != 0 || cmd_theta != 0) {
                nao.setVelocity(cmd_x, cmd_y * 0.7f, cmd_theta);
            }

            last_cmd = now;
        }

        // Display frame with status
        frame_count++;
        if(frame_count % 30 == 0)
        {
            cout << "Frame: " << frame_count << " | Track: " << (ttrack*1000) << "ms";
            if (cmd_x != 0 || cmd_y != 0 || cmd_theta != 0) {
                cout << " | Moving: x=" << cmd_x << " y=" << cmd_y << " th=" << cmd_theta;
            }
            cout << endl;
        }

        // Show frame
        cv::imshow("NAO SLAM + Control", frame_bgr);
    }

    // Stop robot
    cout << "Stopping robot..." << endl;
    nao.setVelocity(0, 0, 0);

    // Stop all threads
    SLAM.Shutdown();

    // Save trajectory
    cout << "Saving trajectory..." << endl;
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    cout << "Done!" << endl;
    return 0;
}
