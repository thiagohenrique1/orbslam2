/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<iomanip>

#include<opencv2/core/core.hpp>

#include"System.h"

using namespace std;

void LoadImages(const string &strSequence, vector<string> &vstrImageFilenames, vector<string> &motionMasks,
                vector<double> &vTimestamps);

int main(int argc, char **argv)
{
    std::cout << "STARTING ORB-SLAM2" << std::endl;
    if(argc != 4)
    {
        cerr << endl << "Usage: ./mono_kitti path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<string> motionMasks;
    vector<double> vTimestamps;
    LoadImages(string(argv[3]), vstrImageFilenames, motionMasks, vTimestamps);
    std::cout << "Found " << vTimestamps.size() << " timestamps" << std::endl;
    std::cout << "Found " << vstrImageFilenames.size() << " images" << std::endl;

    int nImages = vstrImageFilenames.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;


    bool filter = true;
    std::string path = string(argv[3]);
    std::string sequence = path.substr(path.find_last_of("/") + 1);

    // Main loop
    cv::Mat im;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image from file
        im = cv::imread(vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        std::string mask_path = motionMasks[ni];
        cv::Mat mask;
        if (!mask_path.empty()) {
            mask = cv::imread(mask_path, CV_LOAD_IMAGE_GRAYSCALE);
        }

        if(im.empty())
        {
            cerr << endl << "Failed to load image at: " << vstrImageFilenames[ni] << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        if (!filter) {
            mask = cv::Mat();
        }
        SLAM.TrackMonocular(im,mask,tframe);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    if (filter) {
        SLAM.SaveKeyFrameTrajectoryTUM("/root/whuvid/orbslam/filtered_" + sequence + ".txt");
    } else {
        SLAM.SaveKeyFrameTrajectoryTUM("/root/whuvid/orbslam/unfiltered_" + sequence + ".txt");
    }

    return 0;
}

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageFilenames, vector<string> &motionMasks,
        vector<double> &vTimestamps) {
    std::string image_list_path = strPathToSequence + "/other_files/image.txt";
    std::string mask_location = strPathToSequence + "/pred";

    std::ifstream image_list(image_list_path);
    std::string line;
    // skip first 3 lines
    std::getline(image_list, line);
    std::getline(image_list, line);
    std::getline(image_list, line);
    // int i = 0;
    while(std::getline(image_list, line)) {
        // fs::path image_path = strPathToSequence + "/" + line;
        // long timestamp_long = std::stol(image_path.stem());
        // // only add if file exists
        // if (fs::exists(image_path)) {
        //     vstrImageFilenames.push_back(image_path.string());
        //     vTimestamps.push_back(timestamp_long / 1e6);
        // }

        // without using filesystem
        std::string image_path = strPathToSequence + "/" + line;
        // check if file exists
        std::ifstream f(image_path.c_str());
        if (!f.good()) {
            std::cerr << "File not found: " << image_path << std::endl;
            continue;
        }
        // std::cout << image_path << std::endl;
        // from the last / to the first .
        std::string timestamp_str = image_path.substr(image_path.find_last_of("/") + 1, image_path.find_first_of(".") - image_path.find_last_of("/") - 1);
        // std::cout << timestamp_str << std::endl;
        long timestamp_long = std::stol(timestamp_str);
        vstrImageFilenames.push_back(image_path);
        vTimestamps.push_back(timestamp_long / 1e9);
        std::string image_filename = image_path.substr(image_path.find_last_of("/") + 1);
        std::string mask_path = mask_location + "/" + image_filename;
        std::ifstream f_mask(mask_path.c_str());
        if (f_mask.good()) {
            motionMasks.push_back(mask_path);
        } else {
            motionMasks.push_back("");
        }
    }
    // for (auto& [timestamp, path] : left_images_path) {
    //     std::string filename = fs::path(path).filename();
    //     std::string mask_path = strPathToSequence + "/pred/" + filename;
    //     if (fs::exists(mask_path)) {
    //         left_masks_path[timestamp] = mask_path;
    //     }
    // }
}
