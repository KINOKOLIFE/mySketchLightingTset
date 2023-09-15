#pragma once
#include "ofApp.h"
class yUtil{
public:
    static Eigen::Matrix4d getrotx(double degree);
    static glm::mat4 getCentroid_offset(ofxAssimpModelLoader &segmentation);
    static Eigen::Matrix4d vector_matrix(cv::Vec3d rvec, cv::Vec3d tvec);
    static void matrix_vector(cv::Vec3d &rvec, cv::Vec3d &tvec, Eigen::Matrix4d H);
    static void draw_model(ofFbo &fbo, ofEasyCam &cam, ofxAssimpModelLoader &model, Eigen::Matrix4d &mat);
    static void draw_field(ofFbo &fbo,ofCamera &cam);
};


