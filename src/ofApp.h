#pragma once

#include "ofMain.h"
#include "ofxAssimpModelLoader.h"
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/SVD>

#include "yUtil.h"
#include "ofxImGui.h"
#include "video_capture.h"
#include "graphics.h"
#include "realsense.h"
#include "hid.h"
#include "manipulator3d.h"



class ofApp : public ofBaseApp{

public:
    void setup();
    void update();
    void draw();

    void keyPressed  (int key);
    void keyReleased(int key);
    void mouseMoved(int x, int y );
    void mouseDragged(int x, int y, int button);
    void mousePressed(int x, int y, int button);
    void mouseReleased(int x, int y, int button);
    void mouseEntered(int x, int y);
    void mouseExited(int x, int y);
    void windowResized(int w, int h);
    void dragEvent(ofDragInfo dragInfo);
    void gotMessage(ofMessage msg);
    void mouseScrolled(ofMouseEventArgs& mouse);
    void exit();
    
    ofLight pointLight;
    ofLight spotLight;
    ofLight directionalLight;
    
    ofMaterial material;
    ofImage ofLogoImage;
    
    float radius;
    glm::vec3 center;
    bool bShiny;
    bool bSmoothLighting;
    bool bPointLight, bSpotLight, bDirLight;
    bool bUseTexture;
    
    //--imGUI
    ofxImGui::Gui gui;
    void gui_draw();
    //-------3d manipulator
    Manipulator3d mygizmo;
    Manipulator3d minpulator2;
    //-------hidapi
    HID_camera *hid_camera;
    //--videocapture
    ofImage endoscope_image;
    capture_thread endoscope_capture;
    //----realsense
    rs265 real_sense;
    ofxAssimpModelLoader realsense_model;
    //
    ofFbo virtual_image;
    ofCamera endoscope_camera;
};
