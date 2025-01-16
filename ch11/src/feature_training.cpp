#include <iostream>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <DBoW3.h>

// Generate Vocabulary from list of images

int main(int argc, char **argv){
    // read images
    std::cout << "Reading Images: " << std::endl;
    std::vector<cv::Mat> images;
    for (int i=1; i<=10; i++){
        std::string path = "./data/" + std::to_string(i)+".png";
        images.push_back(cv::imread(path));
    }

    // detect ORB features and store it!
    std::cout << "Detecting ORB features: " << std::endl;
    cv::Ptr<cv::Feature2D> detector = cv::ORB::create();
    std::vector<cv::Mat> descriptors;
    for(cv::Mat &img:images){
        std::vector<cv::KeyPoint> kps;
        cv::Mat descriptor;
        detector->detectAndCompute(img, cv::Mat(), kps, descriptor);
        descriptors.push_back(descriptor);
    }

    // create vocabulary
    std::cout << "Creating Vocabulary: " << std::endl;
    DBoW3::Vocabulary vocab;
    vocab.create(descriptors);
    std::cout << "Vocabulary info: " << vocab << std::endl;
    vocab.save("./data/vocabulary.yml.gz");
    return 0;

}