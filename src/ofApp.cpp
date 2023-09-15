#include "ofApp.h"
ofxAssimpModelLoader segmentation;
float cx,cy,cz;
ofEasyCam cam;
ofFbo perspective;
ofRectangle area_perspective = ofRectangle(0,0,1280,720);

ofLight light;
//https://mtscience8848.blogspot.com/2016/09/openframeworks.htmlから引用
glm::mat4 T_offset(1.0);
glm::mat4 segmentations_translate(1.0);
glm::mat4 segmentations_rotation(1.0);


//--------------------------------------------------------------
void ofApp::setup(){
    Eigen::Matrix4d mm = yUtil::getrotx(10);
    //---HID
    hid_camera = new HID_camera();
    //-------
    ofAddListener(ofEvents().mouseScrolled, this, &ofApp::mouseScrolled);
    //https://forum.openframeworks.cc/t/move-camera-along-y-axis-by-scrolling-the-mouse-wheel/41041
    //---fbo, cam
    perspective.allocate(1280, 720);
    //----gizmo
    mygizmo.setup(area_perspective);
    //------imGUI セットアップ
    gui.setup();
    segmentation.loadModel("/Users/takakiyoshiroshi/Documents/pre_ope_sim/KANNO/Segmentation_.obj",true);
    //segmentation.setScaleNormalization(false);
    T_offset = yUtil::getCentroid_offset(segmentation);

    //-------light setting
    light.enable();
    light.setAmbientColor(ofFloatColor(1,1,1,1.0));
    light.setDiffuseColor(ofFloatColor(1,1,1));
    light.setSpecularColor(ofFloatColor(0.1,0.1,0.1));
}

//--------------------------------------------------------------
void ofApp::update() {
    mygizmo.update(cam);
    
    glm::mat4 dist = mygizmo.current * T_offset  ;
    Eigen::Matrix4d  d = glm_Eigen(dist);
    yUtil::draw_field(perspective, cam);
    yUtil::draw_model(perspective, cam, segmentation, d);
}

//--------------------------------------------------------------
void ofApp::draw(){
    perspective.draw(0,0);
    //--------------
    gui_draw();
}
//--------------------------------------------------------------
void ofApp::gui_draw(){
    gui.begin();{
        ImGui::Begin("HID on Camera");{
            if(ImGui::IsWindowHovered()){
                cam.disableMouseInput();
            }
            if(!hid_camera->running){
                if (ImGui::Button(" connect ")) {
                    if(hid_camera->setup(0xE502, 0xBBAB)){
                        hid_camera->startThread();
                    }
                }
            }else{
                if (ImGui::Button(" discconet ")) {
                    hid_camera->disconnect();
                }
            }
    }ImGui::End();
    }gui.end();
}
//--------------------------------------------------------------
void ofApp::mouseScrolled(ofMouseEventArgs& mouse){
    //std::cout<<mouse.scrollY<<std::endl;
}


//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    mygizmo.enable_gizmo();
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){
    
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){

}
