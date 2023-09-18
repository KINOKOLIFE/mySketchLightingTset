#include "ofApp.h"
ofxAssimpModelLoader segmentation;
//float cx,cy,cz;
ofEasyCam cam, cam2;
ofFbo perspective;
ofFbo perspective2;
ofRectangle area_perspective = ofRectangle(0,0,640,360);
ofRectangle area_perspective2 = ofRectangle(0,360,640,360);
ofRectangle area_virtual_image = ofRectangle(640,0,640,360);
ofLight light;
//https://mtscience8848.blogspot.com/2016/09/openframeworks.htmlから引用
glm::mat4 T_offset(1.0);
glm::mat4 segmentations_position(1.0);
glm::mat4 model_mat(1.0);
glm::mat4 segmentations_position_previous(1.0);
/*glm::mat4 scope_gl = glm::mat4( 0, -1, 0, 0,
               0, 0, -1, 0,
               1, 0, 0, 0,
            -500, 0, 16, 1 );
 */
glm::mat4 scope_gl = glm::mat4( 0, -1, 0, 0,
               0, 0, -1, 0,
               1, 0, 0, 0,
            -180, 0, 16, 1 );//----expeimental
glm::mat4 endoscope_lens;
//--------------------------------------------------------------
void ofApp::setup(){
    /*
    Eigen::Matrix4d rs_scope_gl_cordination; rs_scope_gl_cordination << 0,0,1,0,  -1,0,0,0, 0,-1,0,0, 0,0,0,1;
    Eigen::Matrix4d rs_scope_cv_cordination; rs_scope_cv_cordination << 0,0,-1,0,  -1,0,0,0, 0,1,0,0, 0,0,0,1;
    Eigen::Matrix4d rs_scope_trans; rs_scope_trans << 1,0,0,-500,  0,1,0,0, 0,0,1,16, 0,0,0,1;
    Eigen::Matrix4d scope_gl =  rs_scope_trans * rs_scope_gl_cordination;
    glm::mat4 x = eigen_glm(scope_gl);
    */
    
  
    //---realsense
    real_sense.setup();
    //--
    //realsense_model.loadModel("vrcamera_mockup.obj",true);//--
    realsense_model.loadModel("vrcamera_mockup_short.obj");//---expeimental
    realsense_model.setScaleNormalization(false);
    //---HID
    hid_camera = new HID_camera();
    //-------
    ofAddListener(ofEvents().mouseScrolled, this, &ofApp::mouseScrolled);
    //https://forum.openframeworks.cc/t/move-camera-along-y-axis-by-scrolling-the-mouse-wheel/41041
    //---fbo, cam
    perspective.allocate(640, 360);
    cam.setControlArea(area_perspective);
    perspective2.allocate(640, 360);
    cam2.setControlArea(area_perspective2);
    virtual_image.allocate(1280,720);
    float camera_paramter[] = { 860, 858, 666, 373 };
    yUtil::setCameraParameter(endoscope_camera, camera_paramter, 1280, 720);
    //----gizmo
    mygizmo.setup(area_perspective);
    //------imGUI セットアップ
    gui.setup();
    //----モデル読み込み
    segmentation.loadModel("/Users/takakiyoshiroshi/Documents/pre_ope_sim/KANNO/Segmentation_.obj",true);
    //segmentation.setScaleNormalization(false);
    T_offset = yUtil::getCentroid_offset(segmentation);
    //-------light setting
    light.enable();
    light.setAmbientColor(ofFloatColor(1,1,1,1.0));
    light.setDiffuseColor(ofFloatColor(1,1,1));
    light.setSpecularColor(ofFloatColor(0.1,0.1,0.1));
    //--video capture
    endoscope_capture.get_camera_list();
}

//--------------------------------------------------------------
void ofApp::update() {
    //---video capture
    endoscope_image.setFromPixels(endoscope_capture.pixels);
    //----------
    model_mat = glm::mat4(mygizmo.current) ;//* adiition_rotionmat
    segmentations_position = model_mat  * T_offset  ;
    yUtil::draw_field(perspective, cam);
    yUtil::draw_model(perspective, cam, segmentation, segmentations_position);
    yUtil::draw_model(perspective, cam, realsense_model, real_sense.t265_rawoutput);
    
    //----
    mygizmo.update(cam);
    //------------
    yUtil::draw_field(perspective2, cam2);
    endoscope_lens = real_sense.rs_raw *  scope_gl;

    yUtil::draw_axis(perspective2, cam2, endoscope_lens);
    yUtil::draw_model(perspective2, cam2, segmentation, segmentations_position);
    //-----perspective, easycam, realsense_model, real_sense.t265_rawoutput
    yUtil::draw_model(perspective2, cam2, realsense_model, real_sense.t265_rawoutput);
    //draw_axis(perspective2, cam2, real_sense.t265_rawoutput);
    //-----
    glm::mat4 m = glm::inverse(cam.getModelViewMatrix());
    perspective2.begin();{
        cam2.begin();{
            //----exprimental
            ofPushMatrix();{
                ofMultMatrix(mygizmo.rotation);
                ofSetColor(255,0,0,50);
                ofDrawBox(100);
            }ofPopMatrix();
            ofPushMatrix();{
                ofMultMatrix(mygizmo.current);
                ofSetColor(0,0,255,100);
                //ofDrawBox(100);
            }ofPopMatrix();
            //-----
            ofPushMatrix();
            ofMultMatrix(mygizmo.current);
            ofPopMatrix();
            ofFill();
            ofSetColor(255,0,0);
            ofDrawSphere(mygizmo.rotation_center, 4);
            ofPushMatrix();{
                ofMultMatrix(m);
                ofSetColor(200);
                ofDrawBox(10);
                ofNoFill();
                ofSetColor(10);
                ofDrawBox(10);
                ofSetColor(200);
                ofDrawLine(0,0,0,0,0,-550);
                ofDrawAxis(20);
                ofTranslate(0, 0, -550);
                ofDrawRectangle(-320,-180,640,360);
                ofDrawCircle(mygizmo.currentVec_.x,mygizmo.currentVec_.y,10);
            }ofPopMatrix();
        }cam2.end();
    }perspective2.end();
    //----virrtual cam
    yUtil::draw_field(virtual_image, endoscope_camera, endoscope_lens);
    yUtil::draw_model(virtual_image, endoscope_camera, endoscope_lens, segmentation, segmentations_position);
}

//--------------------------------------------------------------
void ofApp::draw(){
    perspective.draw(area_perspective);
    perspective2.draw(area_perspective2);
    endoscope_image.draw(area_virtual_image);
    virtual_image.draw(area_virtual_image);
    //--------------
    gui_draw();
}
//--------------------------------------------------------------
void ofApp::gui_draw(){
    gui.begin();{
        ImGui::Begin("3d mouse");{
            if(ImGui::IsWindowHovered()){
                cam.disableMouseInput();
            }
            if (ImGui::Button("R")) {
                
            }
            if (ImGui::Button("T")) {
                
            }
            if (ImGui::Button("set_position")) {
               // mygizmo.enable_gizmo();
                auto t = glm::translate( glm::mat4(), glm::vec3( 0, 0, -200 ) );
                mygizmo.current = endoscope_lens * t;
                mygizmo.previous = endoscope_lens * t;
            }
        }ImGui::End();
        ImGui::Begin("3d manipulator");{
            if(ImGui::IsWindowHovered()){
                cam.disableMouseInput();
            }
            if(!mygizmo.enable){
                if (ImGui::Button("rotation")) {
                    mygizmo.enable_gizmo();
                }
            }else{
                if (ImGui::Button("easycam")) {
                    mygizmo.enable_gizmo();
                }
            }
        }ImGui::End();
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
        ImGui::Begin("camera select");{
            if(ImGui::IsWindowHovered()){
                cam.disableMouseInput();
            }else{
                
            }
            const char* listbox_items[endoscope_capture.devices_list.size()];
            for(size_t i = 0; i < endoscope_capture.devices_list.size(); i++){
                    listbox_items[i] = endoscope_capture.devices_list[i].c_str();//https://www.sejuku.net/blog/52403
            }
            //https://qiita.com/ousttrue/items/ae7c8d5715adffc5b1fa
            static int listbox_item_current = 0;
            if(ImGui::ListBox("", &listbox_item_current, listbox_items, IM_ARRAYSIZE(listbox_items), 3)){
                string s = listbox_items[listbox_item_current];
                if( std::equal(s.begin() , s.begin() + 7, "H264 USB") ){
                    endoscope_capture.set_param(listbox_item_current,1280,720);
                }
                if( std::equal(s.begin() , s.begin() + 9, "FaceTime HD") ){
                    endoscope_capture.set_param(listbox_item_current,1280,720);
                }
                if( std::equal(s.begin() , s.begin() + 10, "USB3 Video") ){
                    endoscope_capture.set_param(listbox_item_current,1280,720);
                }
                if(!endoscope_capture.isThreadRunning()){
                    endoscope_capture.startThread();
                }
            }
            if(endoscope_capture.setup){
                if (ImGui::Button(" stop camera ")) {
                    endoscope_capture.stop_cam();
                }
            }
            ImGui::Begin("hello realsense");{
                if(ImGui::IsWindowHovered()){
                    //viewer.hover = true; //
                    cam.disableMouseInput();
                }
                if (ImGui::Button(" connect ")) {
                    if(real_sense.connect()){
                        real_sense.startThread();
                    };
                }
                if (ImGui::Button(" disconnect ")) {
                    real_sense.disconnect();
                }
            }ImGui::End();
        }ImGui::End();
    }gui.end();
}
//--------------------------------------------------------------
void ofApp::mouseScrolled(ofMouseEventArgs& mouse){
    //std::cout<<mouse.scrollY<<std::endl;
}
//--------------------------------------------------------------
void ofApp::exit(){
    endoscope_capture.stopThread();
    real_sense.disconnect();
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
