#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include <DBoW3.h>

int main(int argc, char **argv){
    // reading vocabulary from saved path
    DBoW3::Vocabulary vocab("./data/vocabulary.yml.gz");

    // reading images
    std::vector<cv::Mat> images;
    for(int i=0; i<10; i++){
        std::string path = "./data/" + std::to_string(i+1) + ".png";
        images.push_back(cv::imread(path));
    }

    // calculate ORB vector for 
    std::vector<cv::Mat> descriptors;
    cv::Ptr<cv::Feature2D> detector = cv::ORB::create();
    for(cv::Mat &img:images){
        std::vector<cv::KeyPoint> kps;
        cv::Mat desc;
        detector->detectAndCompute(img, cv::Mat(), kps, desc);
        descriptors.push_back(desc);
    }

    // get vector from vocab for given image and calculate score
    std::cout << "Calculating Scores...." << std::endl;
    for(size_t i=0; i<images.size(); i++){
        DBoW3::BowVector v1, v2;
        vocab.transform(descriptors[i], v1);
        for(size_t j=i; j<images.size(); j++){
            vocab.transform(descriptors[j], v2);
            double score = vocab.score(v1, v2);
            std::cout << "Image: " << i << " and image: " << j << ": " << score << std::endl; 
        }
    }

    // create database and retrieve nearest 4 query:
    std::cout << "Comparing images with Database..." << std::endl;
    DBoW3::Database database(vocab, false, 0);
    for(size_t i=0; i<images.size(); i++) database.add(descriptors[i]);
    std::cout << "database info: " << database << std::endl;

    for(size_t i=0; i<images.size(); i++){
        DBoW3::QueryResults ret;
        database.query(descriptors[i], ret, 4);
        std::cout << "searching for image: " << i << " returns " << ret << std::endl;
    }
    
    std::cout << "Done.." << std::endl;
    return 0;
}