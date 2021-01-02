#include "MyPhysicsEngine.h"

namespace GMUCS425
{

    void MyPhysicsEngine::step(float h)
    {
        vector<State> states, new_states;
        particles_to_states(states);
        new_states.resize(states.size());
        step(h, states, new_states);
        states_to_particles(new_states);
    }

    //step forward by h secs
    void MyPhysicsEngine::step(float h, const vector<State>& states, vector<State>& new_states)
    {
        vector<Vector2d> forces;
        vector<dState> dxdv;

        //1. update forces
        for (MyParticle* p : m_particles)
        {
            p->compute_force();
            forces.push_back(p->force);
        }

        //2. compute derivatives
        dxdv.resize(states.size());
        derive(states, forces, dxdv);

        //3. solve ode to get new states
        ode(h, states, dxdv, new_states);
    }

    //TODO: compute the derivatives
    void MyPhysicsEngine::derive(const vector<MyPhysicsEngine::State>& states,
        const vector<Vector2d>& forces,
        vector<MyPhysicsEngine::dState>& dxdv)
    {
        int i = 0;
        for (MyParticle* p : m_particles) {
            dxdv[i].dx = p->vel;
            dxdv[i].dv = p->force / p->mass;
            i++;
        }
        //implement this function
    }

    //TODO: implement midpoint method
    void MyPhysicsEngine::ode(float h,
        const vector<MyPhysicsEngine::State>& states,
        const vector<MyPhysicsEngine::dState>& dxdv,
        vector<MyPhysicsEngine::State>& new_states)
    {
        vector<Vector2d> forces;
        vector<dState> state=dxdv;
        for (MyParticle *p: m_particles)
        {
            p->compute_force();
            forces.push_back(p->force);
        }
        euler(h / 2, states, state, new_states);
        state.resize(states.size());
        derive(states, forces, state);
        euler(h, states, dxdv, new_states); //remove this line
        return;
    }

    //TODO: implement Euler's method
    void MyPhysicsEngine::euler(float h,
        const vector<MyPhysicsEngine::State>& states,
        const vector<MyPhysicsEngine::dState>& dxdv,
        vector<MyPhysicsEngine::State>& new_states)
    {
        
        for (int i = 0; i < states.size(); i++)
        {
            MyPhysicsEngine::State blo = states[i];
            MyPhysicsEngine::dState bio = dxdv[i];
            new_states[i]=setState(states[i].x + (dxdv[i].dx*(h)), states[i].v + (dxdv[i].dv*(h)));
        }
    

    }

}//end namespace GMUCS425
