#include <node.h>

// C standard library
#include <cstdlib>
#include <ctime>

#include <string.h>

#include <opencv2/opencv.hpp>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/features2d.hpp"

using namespace v8;

/**
* The detection function for the
*
* @param string needleImage The image we're looking for
* @param string haystackImage The image we're looking in 
* @param double sensitivity The sensitivity of the feature detector
* @param double minDistance The minimum distance between points
* @param double pointThreshold The number of good points required to determine a match
*/
bool detect(std::string needleImage, std::string haystackImage, double sensitivity, double minDistance, double pointThreshold){

    //Image we're looking for
    cv::Mat object = imread(needleImage,  cv::IMREAD_GRAYSCALE);

    //Image we're looking in
    cv::Mat image = imread(haystackImage, cv::IMREAD_GRAYSCALE );
        
    //Detector setup    
    cv::SurfFeatureDetector detector(sensitivity, 20, 4);
    std::vector<cv::KeyPoint> kp_object;
    
    detector.detect( object, kp_object );
    
    //Calculate descriptors (feature vectors)
    cv::SurfDescriptorExtractor extractor;
    cv::Mat des_object;
    
    extractor.compute( object, kp_object, des_object );
    
    cv::FlannBasedMatcher matcher;
        
    std::vector<cv::Point2f> obj_corners(4);
    
    //Get the corners from the object
    obj_corners[0] = cvPoint(0,0);
    obj_corners[1] = cvPoint( object.cols, 0 );
    obj_corners[2] = cvPoint( object.cols, object.rows );
    obj_corners[3] = cvPoint( 0, object.rows );
                
    cv::Mat des_image, img_matches;
    std::vector<cv::KeyPoint> kp_image;
    std::vector<cv::vector<cv::DMatch > > matches;
    std::vector<cv::DMatch > good_matches;
    std::vector<cv::Point2f> obj;
    std::vector<cv::Point2f> scene;
    std::vector<cv::Point2f> scene_corners(4);
    cv::Mat H;

    detector.detect( image, kp_image );
    extractor.compute( image, kp_image, des_image );
    
    matcher.knnMatch(des_object, des_image, matches, 2);
    
    for(int i = 0; i < std::min(des_image.rows-1,(int) matches.size()); i++){
        if((matches[i][0].distance < .6*(matches[i][1].distance)) && ((int) matches[i].size()<=2 && (int) matches[i].size()>0)){
            good_matches.push_back(matches[i][0]);
        }
    }
                
    //Compare matches verse match threshold
    if (good_matches.size() >= pointThreshold){
        return true;                   
    }else{
        return false;    
    }
                        
    return false;
}


// Function returns whether or not an object was detected
Handle<Value> DetectObject(const Arguments& args) {
    
    HandleScope scope;
    
    //Check for the right number of arguments
    if (args.Length() != 5) {
        ThrowException(Exception::TypeError(String::New("Wrong number of arguments")));
        return scope.Close(Undefined());
    }
    
    //Check that the arguments are of the correct type
    if (!args[0]->IsString() && !args[1]->IsString()) {
        ThrowException(Exception::TypeError(String::New("Arguments are not of the correct type.")));
        return scope.Close(Undefined());
    }

    v8::String::Utf8Value param1(args[0]->ToString());
    v8::String::Utf8Value param2(args[1]->ToString());
    Local<Integer> param3 = args[2]->ToInteger();
    Local<Integer> param4 = args[3]->ToInteger();
    Local<Integer> param5 = args[4]->ToInteger();

    //Convert the parameters to the correct types
    std::string needle = std::string(*param1);
    std::string haystack = std::string(*param2);
    int32_t sensitivity = param3->Value();
    int32_t minDistance = param4->Value();
    int32_t pointThreshold = param5->Value();

    bool found =  detect(needle, haystack, sensitivity, minDistance, pointThreshold);
    
    return scope.Close(Boolean::New(found)); 
}

void RegisterModule(Handle<Object> target) {
    srand(time(NULL));

    // target is the module object you see when require()ing the .node file.
    target->Set(String::NewSymbol("DetectObject"),
        FunctionTemplate::New(DetectObject)->GetFunction());
}

NODE_MODULE(modulename, RegisterModule);
