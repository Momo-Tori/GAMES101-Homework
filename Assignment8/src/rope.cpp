#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

const double damping_factor=0.00005;
namespace CGL
{

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        // TODO: (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.
        Vector2D delta = (end - start) / (num_nodes - 1);
        masses.push_back(new Mass(start, node_mass, false));
        start += delta;
        for (size_t i = 1; i < num_nodes; i++)
        {
            masses.push_back(new Mass(start, node_mass, false));
            springs.push_back(new Spring(masses[i], masses[i - 1], k));
            start += delta;
        }

        //        Comment-in this part when you implement the constructor
        for (auto &i : pinned_nodes)
        {
            masses[i]->pinned = true;
        }
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            auto m1 = s->m1;
            auto m2 = s->m2;
            auto deltaX = m2->position - m1->position;
            Vector2D f = -(s->k) * (deltaX.norm() - s->rest_length) * deltaX / deltaX.norm();
            m1->forces += -f;
            m2->forces += f;
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                auto a = m->forces / m->mass;
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position

                m->velocity = m->velocity + a * delta_t;
                m->position = m->position + m->velocity * delta_t;
                // TODO: (Part 2): Add global damping
            }

            // Reset all forces on each mass
            m->forces = gravity;
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            auto m1 = s->m1;
            auto m2 = s->m2;
            auto deltaX = m2->position - m1->position;
            Vector2D f = -(s->k) * (deltaX.norm() - s->rest_length) * deltaX / deltaX.norm();
            m1->forces += -f;
            m2->forces += f;
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                Vector2D temp_position = m->position;

                auto a = m->forces / m->mass;

                m->position = m->position +(1-damping_factor)*(m->position- m->last_position) + a * delta_t * delta_t;

                m->last_position = temp_position;

                m->forces = gravity;

                // TODO (Part 3.1): Set the new position of the rope mass

                // TODO (Part 4): Add global Verlet damping
            }
        }
    }
}
