#pragma once
#include "ofApp.h"
class yUtil{
public:
    static glm::mat4 getCentroid_offset(ofxAssimpModelLoader &segmentation);
    static Eigen::Matrix4d vector_matrix(cv::Vec3d rvec, cv::Vec3d tvec);
    static void matrix_vector(cv::Vec3d &rvec, cv::Vec3d &tvec, Eigen::Matrix4d H);
    static void draw_model(ofFbo &fbo, ofEasyCam &cam, ofxAssimpModelLoader &model, Eigen::Matrix4d &mat);
    static void draw_model(ofFbo &fbo, ofEasyCam &cam, ofxAssimpModelLoader &model, glm::mat4 &mat);
    static void draw_model(ofFbo &fbo, ofCamera &cam,  glm::mat4 &cam_position, ofxAssimpModelLoader &model, glm::mat4 &model_position);
    static void draw_field(ofFbo &fbo,ofCamera &cam);
    static void draw_field(ofFbo &fbo, ofCamera &cam,  glm::mat4 &cam_position);
    static void setCameraParameter(ofCamera &camera, cv::Mat &intrinscic, float camWidth, float camHeight);
    static void setCameraParameter(ofCamera &camera, float* param, float camWidth, float camHeight);
    static glm::mat4x4 eigen_glm(Eigen::Matrix4d &mat);
    static Eigen::Matrix4d glm_Eigen(glm::mat4x4 &mat);
    static void draw_axis(ofFbo &fbo, ofEasyCam &cam, glm::mat4 &mat);
};

