#include <iostream>
#include <opencv2/opencv.hpp>

int main(int argc, char **argv){

    // check if 3 args there:
    if(argc!=3){
        std::cerr << "Usage: orb_cv image1 image2" << std::endl;
        return 1;
    }

    // read imgaes
    cv::Mat img1 = cv::imread(argv[1], cv::IMREAD_COLOR);
    cv::Mat img2 = cv::imread(argv[2], cv::IMREAD_COLOR);
    assert(img1.data != nullptr && img2.data != nullptr);

    // initialization. kps, descriptors
    std::vector<cv::KeyPoint> kps1, kps2;
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
    std::cout << "Time take for extrating descriptors: " << time_diff.count() << std::endl;

    // draw kps:
    cv::Mat outImg1;
    cv::drawKeypoints(img1, kps1, outImg1, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
    cv::imshow("KeyPoints", outImg1);

    // use Hamming distance to match the features
    std::vector<cv::DMatch> matches;
    t1 = std::chrono::steady_clock::now();
    matcher->match(desc1, desc2, matches);
    t2 = std::chrono::steady_clock::now();
    time_diff = t2-t1;
    std::cout << "Time take for matching descriptors: " << time_diff.count() << std::endl;

    // remove outliers based on distance:
    auto min_max = std::minmax_element(matches.begin(), matches.end(),
                    // this line seems like lambda function in python
                    [](const cv::DMatch &m1, const cv::DMatch &m2){return m1.distance < m2.distance;}
                    );     
    double min_dist = min_max.first->distance;
    double max_dist = min_max.second->distance;
    printf("\t Max dist: %f \n", max_dist);
    printf("\t Min dist: %f \n", min_dist);

    // remove bad matches
    std::vector<cv::DMatch> good_matches;
    for(size_t i=0; i<matches.size(); i++){
        if(matches[i].distance <= std::max(2*min_dist, 30.0)){
            good_matches.push_back(matches[i]);
        }
    }

    // draw results
    cv::Mat img_match, img_goodMatch;
    cv::drawMatches(img1, kps1, img2, kps2, matches, img_match);
    cv::drawMatches(img1, kps1, img2, kps2, good_matches, img_goodMatch);
    cv::imshow("Matches", img_match);
    cv::imshow("Good Matches", img_goodMatch);
    cv::waitKey(0);

    return 0;

}