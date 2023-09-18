//
//  yUtil.cpp
//  mySketchLightingTset
//
//  Created by 高木偉博 on 2023/09/15.
//

#include "yUtil.h"

glm::mat4 yUtil::getCentroid_offset(ofxAssimpModelLoader &segmentation){
    segmentation.setScaleNormalization(false);
    int meshCount = segmentation.getMeshCount();
    ofPoint C;
    int total_vertex = 0;
    for(int i = 0; i < meshCount; i++){
        ofMesh m = segmentation.getMesh(i);
        int nV = m.getNumVertices();
    //The number of the triangles
        int nT = m.getNumIndices();
        for( int n = 0; n < nV; n++){
            C.x += m.getVertex(n).x;
            C.y += m.getVertex(n).y;
            C.z += m.getVertex(n).z;
            total_vertex ++;
        }
    }
    C.x /= total_vertex;
    C.y /= total_vertex;
    C.z /= total_vertex;
    //meshとmodekではx,y座標が反転。
    glm::mat4 T_offset = glm::translate( glm::mat4(), glm::vec3( C.x, C.y, -C.z ) );
    return T_offset;
}
//-----------------------------------------------------
Eigen::Matrix4d yUtil::vector_matrix(cv::Vec3d rvec, cv::Vec3d tvec){
    Eigen::Matrix4d H = Eigen::Matrix4d::Identity();
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    for (unsigned int m = 0; m < 3; m++){
        for (unsigned int n = 0; n < 3; n++){
            H(m, n) = R.at<double>(m, n);
        }
    }
    for (unsigned int m = 0; m < 3; m++){
        H(m, 3) = tvec(m);
    }
    return H;
}
void yUtil::matrix_vector(cv::Vec3d &rvec, cv::Vec3d &tvec, Eigen::Matrix4d H){
    cv::Vec3d temp1, temp2;
    cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
    for (unsigned int m = 0; m < 3; m++){
        for (unsigned int n = 0; n < 3; n++){
            R.at<double>(m, n) = H(m, n);
        }
    }
    cv::Rodrigues(R, temp1);
    for (unsigned int m = 0; m < 3; m++){
        temp2(m) = H(m, 3);
    }
    rvec = temp1;
    tvec = temp2;
}
//----------------------------------------------------------
void yUtil::draw_model(ofFbo &fbo, ofEasyCam &cam, ofxAssimpModelLoader &model, glm::mat4 &mat){
    ofEnableDepthTest();
    fbo.begin();{
        cam.begin();{
            ofLight    light;
            light.enable();
            //light.setSpotlight();
            light.setAmbientColor(ofFloatColor(1.0,1.0,1.0,1.0));
            light.setDiffuseColor(ofFloatColor(1.0,1.0,1.0));
            light.setSpecularColor(ofFloatColor(0.1,0.1,0.1));//important
          
            glm::mat4 m = cam.getModelViewMatrix();
            glm::mat4 m_ = glm::inverse(m);
            light.setPosition(m_[3][0], m_[3][1], m_[3][2]);
            ofPushMatrix();{
                ofMultMatrix(mat);
                model.drawFaces();
            }ofPopMatrix();
        }cam.end();
    }fbo.end();
    ofDisableDepthTest();
};
void yUtil::draw_model(ofFbo &fbo, ofCamera &cam,  glm::mat4 &cam_position, ofxAssimpModelLoader &model, glm::mat4 &model_position){
    ofEnableDepthTest();
    fbo.begin();{
        cam.begin();{
            ofLight    light;
            light.enable();
            //light.setSpotlight();
            light.setAmbientColor(ofFloatColor(1.0,1.0,1.0,1.0));
            light.setDiffuseColor(ofFloatColor(1.0,1.0,1.0));
            light.setSpecularColor(ofFloatColor(0.1,0.1,0.1));//important
            glm::mat4 m = cam.getModelViewMatrix();
            glm::mat4 m_ = glm::inverse(m);
            light.setPosition(m_[3][0], m_[3][1], m_[3][2]);
            ofPushMatrix();{
                glm::mat4 c_i = glm::inverse(cam_position);
                ofMultMatrix(c_i);
                ofPushMatrix();{
                    ofMultMatrix(model_position);
                    model.drawFaces();
                }ofPopMatrix();
            }ofPopMatrix();
        }cam.end();
    }fbo.end();
    ofDisableDepthTest();
};
void yUtil::draw_model(ofFbo &fbo, ofEasyCam &cam, ofxAssimpModelLoader &model, Eigen::Matrix4d &mat){
    ofEnableDepthTest();
    fbo.begin();{
        cam.begin();{  
            ofLight    light;
            light.enable();
            //light.setSpotlight();
            light.setAmbientColor(ofFloatColor(1.0,1.0,1.0,1.0));
            light.setDiffuseColor(ofFloatColor(1.0,1.0,1.0));
            light.setSpecularColor(ofFloatColor(0.1,0.1,0.1));//important
          
            glm::mat4 m = cam.getModelViewMatrix();
            glm::mat4 m_ = glm::inverse(m);
            light.setPosition(m_[3][0], m_[3][1], m_[3][2]);
            ofPushMatrix();{
                ofMultMatrix(eigen_glm(mat));
                model.drawFaces();
            }ofPopMatrix();
        }cam.end();
    }fbo.end();
    ofDisableDepthTest();
};
//----------------------------------------------------------
void yUtil::draw_field(ofFbo &fbo,ofCamera &cam){
    ofEnableDepthTest();
    fbo.begin();
        ofClear(0);
    //ofBackground(100);
        ofBackgroundGradient(ofColor(64), ofColor(0));
        cam.begin();
            ofDrawGrid(10,10,true,false,false,true);
            ofDrawAxis(10);
        cam.end();
    fbo.end();
};
void yUtil::draw_field(ofFbo &fbo, ofCamera &cam, glm::mat4 &cam_position){
    ofEnableDepthTest();
    fbo.begin();
        ofClear(0);
    //ofBackground(100);
        ofBackgroundGradient(ofColor(64), ofColor(0));
        cam.begin();
            ofPushMatrix();
                glm::mat4 c_i = glm::inverse(cam_position);
                ofMultMatrix(c_i);
                ofDrawGrid(10,10,true,false,false,true);
                ofDrawAxis(10);
            ofPopMatrix();
        cam.end();
    fbo.end();
};

//---------------------------------------------------------
void yUtil::setCameraParameter(ofCamera &camera, cv::Mat &intrinscic, float camWidth, float camHeight){
    float fov = 2.0 * atan( camHeight /2.0 / (float)intrinscic.at<double>(1,1)) / M_PI * 180.0;
    camera.setFov(fov);// screen height /2.0   :   fy
    float shift_x = 1.0 - 2.0 * (float)intrinscic.at<double>(0,2) / camWidth;
    float shift_y = 2.0 * (float)intrinscic.at<double>(1,2) / camHeight - 1.0;
    camera.setLensOffset(glm::vec2(shift_x, shift_y ));
    
}
void yUtil::setCameraParameter(ofCamera &camera, float* param, float camWidth, float camHeight){
    //param = {fx,fy, cx, cy}
    float fov = 2.0 * atan( camHeight /2.0 / param[1]) / M_PI * 180.0;
    camera.setFov(fov);// screen height /2.0   :   fy
    float shift_x = 1.0 - 2.0 * param[2] / camWidth;
    float shift_y = 2.0 * param[3] / camHeight - 1.0;
    camera.setLensOffset(glm::vec2(shift_x, shift_y ));
};
//-------------------------------------------------------------
glm::mat4x4 yUtil::eigen_glm(Eigen::Matrix4d &mat){
    ofMatrix4x4 dest;
    float array[16];
    for(int j=0; j<4 ; j++){
        for(int k=0; k<4; k++){
            array[j*4 + k] = (float)mat(j,k);
        }
    }
    dest.set(array);
    ofMatrix4x4 src = dest.getTransposedOf(dest);
    return glm::mat4x4(src);
};
Eigen::Matrix4d yUtil::glm_Eigen(glm::mat4x4  &mat){
    Eigen::Matrix4d dest;
    for(int j=0; j<4 ; j++){
        for(int k=0; k<4; k++){
            dest(j,k) = mat[j][k];
        }
    }
    return dest.transpose();
};
//---------------------------------------------------------------------
void yUtil::draw_axis(ofFbo &fbo, ofEasyCam &cam, glm::mat4 &mat){
    fbo.begin();{
        cam.begin();{
            ofPushMatrix();{
                ofMultMatrix(mat);
                ofDrawAxis(100);
            }ofPopMatrix();
        }cam.end();
    }fbo.end();
};

