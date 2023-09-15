//
//  yUtil.cpp
//  mySketchLightingTset
//
//  Created by 高木偉博 on 2023/09/15.
//

#include "yUtil.h"
Eigen::Matrix4d yUtil::getrotx(double degree)
{
    Eigen::Matrix4d R = Eigen::Matrix4d::Identity(4,4);
    Eigen::Matrix3d AxisAngle;
    Eigen::Vector3d axis;
    axis<<1,0,0;  //x軸を指定
    AxisAngle = Eigen::AngleAxisd( degree / 180.0 * M_PI, axis);
    R.block(0,0,3,3) = AxisAngle;
    return R;
}
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
