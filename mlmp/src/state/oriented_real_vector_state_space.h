#ifndef OMPL_BASE_SPACES_REAL_VECTOR_STATE_SPACE_
#define OMPL_BASE_SPACES_REAL_VECTOR_STATE_SPACE_

#include "ompl/base/StateSpace.h"
#include "ompl/base/spaces/RealVectorBounds.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include <vector>
#include <string>
#include <map>

namespace ompl {
    namespace base {

        /** \brief A state space representing R<sup>n</sup>, With extra information about the orientation of the stick movment.
         *  The distance function is the L2 norm.
         *  */
        class OrientedRealVectorStateSpace : public RealVectorStateSpace
        {
            OrientedRealVectorStateSpace(int n, int r) : RealVectorStateSpace(n*r), n(n), r(r) {}

            void interpolate (const State *from, const State *to, double t, State *state) {
                const auto *rfrom = static_cast<const StateType *>(from);
                const auto *rto = static_cast<const StateType *>(to);
                const StateType *rstate = static_cast<StateType *>(state);

                srand(static_cast <unsigned int> (time(0)));
                for (auto i=0; i<r; ++i) {
                    for (auto j=0; j<n; ++j) {
                        int dir;
                        if (j == 0) {
                            dir = 2 * (rand() % 2) -1;
                        } else {
                            dir = findDirection(rstate->values[i*n + j -1], rfrom->values[], rto->values[]);
                        }
                        

                    }
                }
            }

            double findDirection(const double prev, const double start, const double goal) {
                if (start < goal) {
                    if (start <= prev && prev <= goal) {
                        return -1;
                    } else {
                        return 1;
                    }
                } else {
                    if (goal <= prev && prev <= start) {
                        return 1;
                    } else {
                        return -1;
                    }
                }
            }

            protected:
            int n;
            int r;
        };
    }
}


#endif