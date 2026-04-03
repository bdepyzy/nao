/**
* NAO Robot SLAM Bridge - Connects MJPEG stream to ORB-SLAM3
*
* Usage: ./mono_nao path_to_vocabulary path_to_settings [stream_url]
* Default stream: http://169.254.81.31:8080/stream
*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<opencv2/core/core.hpp>
#include<opencv2/imgproc.hpp>
#include<opencv2/highgui.hpp>
#include<System.h>

using namespace std;

int main(int argc, char **argv)
{
    if(argc < 3)
    {
        cerr << endl << "Usage: ./mono_nao path_to_vocabulary path_to_settings [stream_url]" << endl;
        cerr << endl << "Example: ./mono_nao ../../Vocabulary/ORBvoc.txt NAO_640x480.yaml" << endl;
        cerr << "         ./mono_nao ../../Vocabulary/ORBvoc.txt NAO_640x480.yaml http://169.254.81.31:8080/stream" << endl;
        return 1;
    }

    // Stream URL - default to NAO robot
    string stream_url = (argc >= 4) ? string(argv[3]) : "http://169.254.81.31:8080/stream";

    cout << endl << "-------" << endl;
    cout << "NAO SLAM Bridge" << endl;
    cout << "Connecting to: " << stream_url << endl;
    cout << "Vocabulary: " << argv[1] << endl;
    cout << "Settings: " << argv[2] << endl;
    cout << "-------" << endl << endl;

    // Open video stream
    cv::VideoCapture cap(stream_url);
    if(!cap.isOpened())
    {
        cerr << "Failed to connect to stream: " << stream_url << endl;
        cerr << "Make sure robot_streamer.py is running on the NAO" << endl;
        return 1;
    }

    cout << "Connected! Starting SLAM..." << endl;

    // Create SLAM system
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, true);
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
        chrono::steady_clock::time_point now = chrono::steady_clock::now();
        timestamp = chrono::duration_cast<chrono::duration<double>>(now - start_time).count();

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

        // Display frame with tracking info
        frame_count++;
        if(frame_count % 30 == 0)
        {
            cout << "Frame: " << frame_count << " | Track time: " << (ttrack*1000) << "ms" << endl;
        }

        // Show frame
        cv::imshow("NAO SLAM", frame_bgr);

        // ESC to quit
        if(cv::waitKey(1) == 27)
        {
            cout << endl << "ESC pressed - shutting down..." << endl;
            break;
        }
    }

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    cout << "Saving trajectory to KeyFrameTrajectory.txt" << endl;
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
    SLAM.SaveTrajectoryTUM("FrameTrajectory.txt");

    cout << "Done!" << endl;
    return 0;
}
