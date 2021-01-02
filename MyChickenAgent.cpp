#include "MyChickenAgent.h"
#include "MyGame.h"
#include "MyDragonAgent.h"
#include "mathtool/intersection.h"
using namespace mathtool;

namespace GMUCS425
{
    //DO NOT CHANGE THIS FUNCTION
    //this function updates this->force (from MyPartcile)
    //implement: 3 flocking force, 1 obstacle avoiding force
    //           1 following force if this chicken is a leader
    void MyChickenAgent::compute_force()
    {
        //DONT CHANGE ANYTHING HERE
        Vector2d boid_force = compute_flocking_force();
        Vector2d obst_force = compute_obstacle_avoiding_force();

        this->force = boid_force + obst_force;

        if (this->leader)
        {
            Vector2d follow_force = compute_following_force();
            this->force += follow_force;
        }

        //add drag force
        this->force += this->vel * (-0.1);
        //DONT CHANGE ANYTHING HERE
    }

    //compute each force
    Vector2d MyChickenAgent::compute_flocking_force()
    {
        Vector2d force_sep, force_align, force_coherent,force_capn;

        list<MyChickenAgent*> neighbors;
        
        this->get_neighbors(neighbors);
        float leadx = 0;
        float leady = 0;
        int nsize = neighbors.size();
        if (nsize == 0) return Vector2d();
        float wsum=0;
        Vector2d xcm;
        //1. compute separation force
        for (MyChickenAgent* a : neighbors) {
            Vector2d d_i = this->pos-a->pos;
            force_sep =force_sep+ (((d_i / sqr(d_i.norm())))*(1/nsize));

            Vector2d xc = Vector2d(a->pos[0], a->pos[1]);
            float w = 1 / (d_i.norm());
            wsum = wsum + w;
          
            if (a->leader) {
                leadx = a->x;
                leady = a->y; 
            }
            
           force_align = force_align + (a->vel*(1/nsize));

           xcm = xcm+ (xc*(1/nsize));
        }
       
        force_align = force_align / nsize;
        xcm = xcm/(nsize);
     
        force_coherent = xcm - (Vector2d)(x, y);
        
      
        //2. compute alignment force
        
       
        //3. compute coherent force
        if (!this->leader) {
            force_capn[0] = leadx - x;
            force_capn[1] = leady - y;
            force_capn = force_capn / k_follow;

        }
        else {
            force_capn = { 0,0 };
        }
        //add the forces together
        return force_sep * k_sep + force_align * k_align + force_coherent * k_coherent+force_capn;
    }

    Vector2d MyChickenAgent::compute_obstacle_avoiding_force()
    {
        Vector2d force;
        Point2d predict_pos = this->pos + this->vel.normalize() * this->view_radius;

        //check if there is collision between this->pos and predict_pos
        //if so, compute the point of collision.
        HalfPlane hp;
        if (checkCollision(this->pos, predict_pos, hp))
        {
            
            Vector2d vl = { this->pos[0],hp.p[1]-this->pos[1]};
            force=1 / sqrt(sqr(vl));
            force = force * hp.n;
            force = force * k_obst;
            //TODO:
            //use hp to determine a force that pushes this chicken
            //away from the obstacle
            //remember to scale the force by k_obst
        }

        return force;
    }

    //DO NOT CHANGE THIS FUNCTION:
    //detect collision between line segment ab and the static obstacles
    //if so, return true and store the point of collision in hp
    bool MyChickenAgent::checkCollision(const Point2d& a, const Point2d& b, HalfPlane& hp)
    {
        //DONT CHANGE ANYTHING HERE
        const std::list<MyAgent* >& agents = getMyGame()->getSceneManager()->get_active_scene()->get_agents();
        mathtool::Box2d box;
        for (auto obst : agents)
        {
            if (obst->is_movable()) continue;
            box.x = obst->getX();
            box.y = obst->getY();
            box.width = obst->getSprite()->getWidth(obst->getScale());
            box.height = obst->getSprite()->getHeight(obst->getScale());

            //check if ab intersects box
            if (checkCollision(box, a, b, hp)) return true;
        }//end obst

        //bounding box of the entire level, prevent the chicken going out of screen
        box.x = 0; box.y = -5;
        box.width = getMyGame()->getScreenWidth(); box.height = 5;
        if (checkCollision(box, a, b, hp)) return true; //TOP

        box.x = -5; box.y = 0;
        box.width = 5; box.height = getMyGame()->getScreenHeight();
        if (checkCollision(box, a, b, hp)) return true; //LEFT

        box.x = 0; box.y = getMyGame()->getScreenHeight();
        box.width = getMyGame()->getScreenWidth(); box.height = 5;
        if (checkCollision(box, a, b, hp)) return true; //BOTTOM

        box.x = getMyGame()->getScreenWidth(); box.y = 0;
        box.width = 5; box.height = getMyGame()->getScreenHeight();
        if (checkCollision(box, a, b, hp)) return true; //RIGHT

        return false;
        //DONT CHANGE ANYTHING HERE
    }


    bool MyChickenAgent::checkCollision(const mathtool::Box2d& box,
        const Point2d& a, const Point2d& b,
        HalfPlane& hp)
    {
        
        double v1[] = { box.x+0.0001, box.y +0.0001};
        double v2[] = { box.x+0.0001, box.y+box.height +0.0001};
        double v3[] = { box.width + box.x+0.0001, box.height + box.y+0.0001 };
        double v4[] = { box.width + box.x+0.0001, box.height+0.0001};
        double aa[] = { a[0]+0.0001,a[1] +0.0001};
        double ba[] = { b[0]+0.01,b[1] +0.01};
        double hpa[2] = { 0.0001,0.001};
        double hpa1[2] = {0.0001,0.0001 };
        double hpa2[2] = {0.0001,0.0001 };
        double hpa3[2] = {0.0001,0.0001 };
        char v1v2 = SegSegInt(v1, v2, ba, aa, hpa);
        char v1v4= SegSegInt(v1, v4, ba, aa, hpa1);
        char v2v3= SegSegInt(v2, v3, ba, aa, hpa2);
        char v3v4= SegSegInt(v3, v4, ba, aa, hpa3);
        //TODO: WORK ON THIS FUNCTION

        if (v1v2 != '0') {
            hp.p[0] = hpa[0];
            hp.p[1] = hpa[1];
            hp.n = {-1,0};

        }
        else if (v1v4 != '0') {
            hp.p[0] = hpa1[0];
            hp.p[1] = hpa1[1];
            hp.n = { 0,-1 };
           
        }
        else if (v2v3 != '0') {
            hp.p[0] = hpa2[0];
            hp.p[1] = hpa2[1];
            hp.n = { 0,1 };
        }
        else if (v3v4 != '0') {
            hp.p[0] = hpa3[0];
            hp.p[1] = hpa3[1];
            hp.n = { 1,0 };
        }
        else {
            return false;
        }
        
        return true;
    }

    //if this chicken is a leader, find a force to drag the chicken to the dragon
    Vector2d MyChickenAgent::compute_following_force()
    {

        MyDragonAgent* dragon = get_dragon();
        assert(dragon);
        Vector2d force;
        
            Vector2d aeb = Vector2d(dragon->getX(), dragon->getY());
            Vector2d tmp = Vector2d(this->pos[0], this->pos[1]);
            Vector2d l = (aeb - tmp);
            Vector2d labs = { ABS(l[0]),ABS(l[1]) };
            Vector2d vl = (this->vel * l);
            force = l.normalize() * (l.norm() * k_follow);
        
        
        //NOTE: DON"T USE MOTION PLANNER;
        //      Simply use the vector from this chiken to dragon to determine the force
        //      Remember to use k_follow to scale the force
        return force;
    }

    //DO NOT CHANGE ANYTHING BELOW

    void MyChickenAgent::display()
    {
        if (!this->visible) return; //not visible...
        //setup positions and ask sprite to draw something
        this->sprite->display(x, y, scale, degree, NULL, this->vel[0] < 0 ? SDL_FLIP_HORIZONTAL : SDL_FLIP_NONE);

        SDL_Renderer* renderer = getMyGame()->getRenderer();
        if (this->leader) SDL_SetRenderDrawColor(renderer, 255, 0, 0, 100);
        else SDL_SetRenderDrawColor(renderer, 200, 200, 200, 100);

        draw_bounding_box();

        if (this->leader) SDL_SetRenderDrawColor(renderer, 255, 0, 0, 100);
        else SDL_SetRenderDrawColor(renderer, 200, 200, 200, 100);
        draw_circle(x, y, view_radius);
    }

    void MyChickenAgent::handle_event(SDL_Event& e)
    {

    }

    //TODO: get a list of nearby neighbors
    void MyChickenAgent::get_neighbors(list<MyChickenAgent*>& neighbors)
    {
        const std::list<MyAgent* >& agents = getMyGame()->getSceneManager()->get_active_scene()->get_agents();
        for (MyAgent* agent : agents)
        {
            if (agent == this) continue;
            MyChickenAgent* other = dynamic_cast<MyChickenAgent*>(agent);
            if (other == NULL) continue; //not a chicken

            if ((this->pos - other->pos).norm() < this->view_radius)
            {
                neighbors.push_back(other);
            }
        }//end for agent
    }

    MyDragonAgent* MyChickenAgent::get_dragon()
    {
        MyDragonAgent* dragon = NULL;

        //find the dragon agent and get its position
        const std::list<MyAgent* >& agents = getMyGame()->getSceneManager()->get_active_scene()->get_agents();
        for (MyAgent* agent : agents)
        {
            if (dynamic_cast<MyDragonAgent*>(agent) == NULL) //not a dragon
                continue;
            dragon = dynamic_cast<MyDragonAgent*>(agent);
            break;
        }

        return dragon;
    }


    void MyChickenAgent::update()
    {
        //update agent position using particle position
        x = this->pos[0];
        y = this->pos[1];
        degree = atan2(this->vel[1], this->vel[0]) * 180 / PI;
        if (this->vel[0] < 0) degree -= 180;
    }

}//end namespace
