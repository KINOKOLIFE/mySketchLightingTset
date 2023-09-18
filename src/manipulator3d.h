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
    glm::vec3 currentVec_;
    glm::vec3 rotation_center;
    //glm::vec3 rotation_center_previos;
    
    float focal_plane = 500; // pixel
    Manipulator3d(){
        
    }
    void setup(ofRectangle &area){
        rotation = glm::mat4(1.0);
        ofRegisterMouseEvents(this);
        px = area.x;
        py = area.y;
        width = area.width;
        height = area.height;
        ofAddListener(ofEvents().mouseScrolled, this, &Manipulator3d::mouseScrolled);
    }
    void setup(ofRectangle &area, ofEasyCam &cam, float cam_width){
        ofRegisterMouseEvents(this);
        px = area.x;
        py = area.y;
        width = area.width;
        height = area.height;
        ofAddListener(ofEvents().mouseScrolled, this, &Manipulator3d::mouseScrolled);
        focal_plane = cam_width / 2.0 / tan((cam.getFov()/2.0) / 180.0 * M_PI);
        //std::cout<<focal_plane<<std::endl;
    }
    void update(ofEasyCam &cam){
        cam_mat = glm::inverse(cam.getModelViewMatrix());
        if(this->enable){
            cam.disableMouseInput();
            cam_mat[3][0] = 0.0;
            cam_mat[3][1] = 0.0;
            cam_mat[3][2] = 0.0;
        }else{
            rotation = glm::mat4(1.0);
            cam.enableMouseInput();
            glm::vec3 o = glm::vec3(this->current * glm::vec4(0.0, 0.0, 0.0, 1.0));
            glm::vec3 c = glm::vec3(cam_mat * glm::vec4(0.0, 0.0, 0.0, 1.0));
            glm::vec3 o_c = o - c;
            float l = length(o_c);
            glm::vec3 zvec = glm::vec3(cam_mat * glm::vec4(0.0, 0.0, -1.0, 1.0));
            glm::vec3 o_c_ = glm::normalize(o_c);
            glm::vec3 zvec_ = glm::normalize(zvec);
            
            float cosThea = glm::dot(o_c_, zvec_);
            rotation_center = glm::vec3(cam_mat * glm::vec4(0.0, 0.0, l * cosThea, 1.0));
        }
    }
    void enable_gizmo(){
        this->enable = !this->enable;
    }
    void mouseMoved(ofMouseEventArgs & args){
    }
    void mouseDragged(ofMouseEventArgs & args){
        if(this->enable && inner(args.x, args.y)){
            glm::vec3 currentVec = glm::vec3 ( -(args.x - px - width /2.0), args.y - py - height /2.0, focal_plane);
            currentVec_ = glm::vec3 ( -(args.x - px - width /2.0), args.y - py - height /2.0, focal_plane);
            if(args.button == 0){
                currentVec = glm::normalize(currentVec);
                startVec = glm::normalize(startVec);
                glm::vec3 a1 = glm::vec3( cam_mat * glm::vec4(currentVec, 1.0));
                glm::vec3 b1 = glm::vec3( cam_mat * glm::vec4(startVec, 1.0));
                
                float dot = glm::dot( a1, b1 );
                float angle = glm::acos(dot);
                 
                glm::vec3 axis = cross(a1, b1);
                axis = glm::normalize(axis);
                auto t = glm::translate( glm::mat4(), rotation_center );
                auto ti = glm::inverse(t);
                auto r = glm::rotate( glm::mat4(), angle , axis);
                rotation = t * r * ti;
                //rotation = glm::rotate( glm::mat4(), angle , axis);
                current =  rotation * previous;
            }
           
            if(args.button == 1){
                glm::vec3 delta = currentVec - startVec;
                delta.x =  - delta.x;
                delta.y =  - delta.y;
                delta.z =  0.0f;
                delta =  glm::vec3( cam_mat * glm::vec4(delta, 1.0));
                rotation = glm::translate( glm::mat4(), delta );
                current = rotation * previous ;
            }
            if(args.button == 2){
                glm::vec3 delta = currentVec - startVec;
                delta.z =  delta.x;
                delta.x =  0.0;
                delta.y =  0.0;
                delta =  glm::vec3( cam_mat * glm::vec4(delta, 1.0));
                rotation = glm::translate( glm::mat4(), delta );
                current = rotation * previous ;
            }
        
        }
    };
    void mousePressed(ofMouseEventArgs & args){
        if(this->enable && inner(args.x, args.y)){
            startVec = glm::vec3 (-(args.x - px - width /2.0), args.y -py  - height /2.0, focal_plane);
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
            auto t = glm::translate( glm::mat4(), rotation_center );
            auto ti = glm::inverse(t);
            auto r = glm::rotate( glm::mat4(),  rad , axis);
            //rotation = glm::rotate( glm::mat4(),  rad , axis);
            rotation = t * r * ti;
            current =  rotation * previous ;
            previous = current;
        }
    };
    void mouseEntered(ofMouseEventArgs & args){};
    void mouseExited(ofMouseEventArgs & args){};
    bool inner(float x, float y){
        return px < x && x < px + width && py < y && y < py + height;
    }
};
