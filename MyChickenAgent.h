#pragma once

#include "MyAgent.h"
#include "MyPathPlanner.h"
#include "MyPhysicsEngine.h"
#include "mathtool/Box.h"

namespace GMUCS425
{
    class MyDragonAgent;

    class MyChickenAgent : public MyAgent, public MyParticle
    {
      public:

        typedef mathtool::Point2d  Point2d;
        typedef mathtool::Vector2d Vector2d;

        MyChickenAgent(bool isleader, float R, float k_sep, float k_coherent, float k_align, float k_obst, float k_follow, float mass)
        :MyAgent(true,true)
        {
          //NEW
          view_radius=R;
          leader=isleader;
          this->mass=mass;
          this->k_sep=k_sep;
          this->k_coherent=k_coherent;
          this->k_align=k_align;
          this->k_obst=k_obst;
          this->k_follow=k_follow;
          planner=NULL;
        }

        virtual void update();
        virtual void display();
        virtual void handle_event(SDL_Event & e);

        virtual void tranlate(float x, float y){
          this->x+=x; this->y+=y;
          this->pos[0]+=x;
          this->pos[1]+=y;
        }

        virtual void tranlateTo(float x, float y){
          this->x=x; this->y=y;
          this->pos[0]=x;
          this->pos[1]=y;
        }

        //TODO: this function updates this->force (from MyPartcile)
        //implement: 3 flocking force, 1 obstacle avoiding force
        //           1 following force if this chicken is a leader
        virtual void compute_force();

      protected:

        //compute each force
        Vector2d compute_flocking_force();
        Vector2d compute_obstacle_avoiding_force();
        Vector2d compute_following_force();

        //collision detection
        struct HalfPlane
        {
          Point2d  p; //a point on the line
          Vector2d n; //normal direction
        };

        //TODO:
        //detect collision between line segment ab and the static obstacles
        //if so, return true and store the point of collision in hp
        bool checkCollision(const Point2d& a, const Point2d& b, HalfPlane& hp);

        bool checkCollision(const mathtool::Box2d& box, const Point2d& a, const Point2d& b, HalfPlane& hp);

      private:

        //get a list of nearby neighbors
        void get_neighbors(list<MyChickenAgent*>& neighbors);

        //find the dragon agent
        MyDragonAgent * get_dragon();

        //flocking boid related variables
        float view_radius; //viewing radius
        float k_sep, k_coherent, k_align; //coefficients for separation, coherence and alignment
        float k_obst, k_follow; //coefficients for obstacle repulsive force, and following force
        bool leader; //is this chicken a leader!

        MyPathPlanner * planner; //this is only used by the leader
    };

}//end namespace
