#pragma once
#include "ofApp.h"

class Manipulator3d{
public:
    float px, py, width, height;
    bool enable = false;;
    
    glm::mat4 previous;
    glm::mat4 current;
    glm::mat4 rotation;
    glm::mat4 cam_mat;
    glm::vec3 startVec;
    Manipulator3d(){
        
    }
    void setup(ofRectangle &area){
        ofRegisterMouseEvents(this);
        px = area.x;
        py = area.y;
        width = area.width;
        height = area.height;
        ofAddListener(ofEvents().mouseScrolled, this, &Manipulator3d::mouseScrolled);
    }
    void update(ofEasyCam &cam){
        if(this->enable){
            cam.disableMouseInput();
            cam_mat = glm::inverse(cam.getModelViewMatrix());
            cam_mat[3][0] = 0.0;
            cam_mat[3][1] = 0.0;
            cam_mat[3][2] = 0.0;
        }else{
            cam.enableMouseInput();
        }
        
    }
    void enable_gizmo(){
        this->enable = !this->enable;
    }
    void mouseMoved(ofMouseEventArgs & args){

    }
    void mouseDragged(ofMouseEventArgs & args){
        if(this->enable && inner(args.x, args.y)){
            glm::vec3 currentVec = glm::vec3 ( -(args.x - px - width /2.0), args.y - py - height /2.0, 200);
            if(args.button == 0){
                currentVec = glm::normalize(currentVec);
                startVec = glm::normalize(startVec);
                glm::vec3 a1 = glm::vec3( cam_mat * glm::vec4(currentVec, 1.0));
                glm::vec3 b1 = glm::vec3( cam_mat * glm::vec4(startVec, 1.0));
                
                float dot = glm::dot( a1, b1 );
                float angle = glm::acos(dot);
                 
                glm::vec3 axis = cross(a1, b1);
                axis = glm::normalize(axis);
                rotation = glm::rotate( glm::mat4(), angle , axis);
                current =  rotation * previous;
            }
           
            if(args.button == 1){
                glm::vec3 delta = currentVec - startVec;
                delta.x =  - delta.x;
                delta.y =  - delta.y;
                delta.z =  0.0f;
                delta =  glm::vec3( cam_mat * glm::vec4(delta, 1.0));
                glm::mat4 T  = glm::translate( glm::mat4(), delta );
                current =   T * previous ;
            }
            if(args.button == 2){
                glm::vec3 delta = currentVec - startVec;
                delta.z =  delta.x;
                delta.x =  0.0;
                delta.y =  0.0;
                delta =  glm::vec3( cam_mat * glm::vec4(delta, 1.0));
                glm::mat4 T = glm::translate( glm::mat4(), delta );
                current =   T * previous ;
            }
        
        }
    };
    void mousePressed(ofMouseEventArgs & args){
        if(this->enable && inner(args.x, args.y)){
            startVec = glm::vec3 (-(args.x - px - width /2.0), args.y -py  - height /2.0, 200);
            //startVec = glm::normalize(startVec);
            current = previous;
        }
    };
    void mouseReleased(ofMouseEventArgs & args){
        if(this->enable && inner(args.x, args.y)){
            previous = current;
        }
    };
    void mouseScrolled(ofMouseEventArgs& args){
        if(this->enable && inner(args.x, args.y)){
            float rad = args.scrollY/180.0 * float(M_PI);
            glm::vec3 axis = glm::vec3( cam_mat * glm::vec4(0,0,1.0, 1.0));
            rotation = glm::rotate( glm::mat4(),  rad , axis);
            current =   rotation * previous ;
            previous = current;
        }
    };
    void mouseEntered(ofMouseEventArgs & args){};
    void mouseExited(ofMouseEventArgs & args){};
    bool inner(float x, float y){
        return px < x && x < px + width && py < y && y < py + height;
    }
};
