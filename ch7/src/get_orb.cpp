#include <iostream>
#include "get_orb.hpp"

void extract_descriptor_matches(cv::Mat img1, cv::Mat img2, 
                                std::vector<cv::KeyPoint> &kps1, std::vector<cv::KeyPoint> &kps2,
                                std::vector<cv::DMatch> &matches){
    cv::Mat desc1, desc2;
    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");

    // detect Oriented FAST
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    detector->detect(img1, kps1);
    detector->detect(img2, kps2);
    // get descriptor
    detector->compute(img1, kps1, desc1);
    detector->compute(img2, kps2, desc2);
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    std::chrono::duration<double> time_diff = t2-t1;
    std::cout << "Time taken for extrating descriptors: " << time_diff.count() << std::endl;

    // use Hamming distance to match the features
    std::vector<cv::DMatch> all_matches;
    t1 = std::chrono::steady_clock::now();
    matcher->match(desc1, desc2, all_matches);
    t2 = std::chrono::steady_clock::now();
    time_diff = t2-t1;
    std::cout << "Time take for matching descriptors: " << time_diff.count() << std::endl;

    // remove outliers based on distance:
    double min_dist = 10000, max_dist = 0;
    for (int i = 0; i < all_matches.size(); i++) {
        double dist = all_matches[i].distance;
        if (dist < min_dist) min_dist = dist;
        if (dist > max_dist) max_dist = dist;
    }

    printf("\t Max dist: %f \n", max_dist);
    printf("\t Min dist: %f \n", min_dist);

    // remove bad matches
    for(size_t i=0; i<desc1.rows; i++){
        if(all_matches[i].distance <= std::max(2*min_dist, 30.0)){
            matches.push_back(all_matches[i]);
        }
    }
    std::cout << "Feature Extraction Done!!" << std::endl;
}
