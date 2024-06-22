#include <math.h>

#include <abstraction_type.h>
#include <scene.h>

#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateSpace.h>
#include <ompl/multilevel/planners/qrrt/QRRT.h>

typedef mlmp::abstraction::AbstractionType AbstractionType;
using namespace ompl::base;

namespace mlmp {
    class CustomValidityChecker : public StateValidityChecker {
        public:
        CustomValidityChecker(const SpaceInformationPtr &si, const mlmp::Scene &scene, int n, int r)
        : StateValidityChecker(si), _scene(scene), _n(n), _r(r)
        {
            si->setStateValidityCheckingResolution(0.001);
        }

        bool isValid(const State *state) const override
        {   
            // return _scene.isStateValid(state, _r, _n);
            return true;
        }

        protected:
        mlmp::Scene _scene;
        int _n;
        int _r;
    };  

    template <typename T>
    PlannerPtr GetMultiLevelPlanner(mlmp::Scene scene, mlmp::abstraction::AbstractionType abstraction, SpaceInformationPtr si, std::string name = "Planner")
    {
        std::vector<SpaceInformationPtr> si_vec;

        switch (abstraction)
            {
            case AbstractionType::Projection:
                for (auto i=1; i<scene.getN(); ++i) {
                    auto iSpace(std::make_shared<ompl::base::RealVectorStateSpace>(scene.getR() * i));
                    ompl::base::RealVectorBounds bounds(scene.getR() * i);
                    bounds.setLow(0);
                    bounds.setHigh(2*M_PI);
                    iSpace->setBounds(bounds);

                    auto iSi = std::make_shared<SpaceInformation>(iSpace);
                    iSi->setStateValidityChecker(std::make_shared<CustomValidityChecker>(iSi, scene, i, scene.getR()));

                    iSpace->setup();
                    si_vec.push_back(iSi);
                }
                break;
            
            case AbstractionType::PriorityOrder:
                for (auto i=1; i<scene.getR(); ++i) {
                    auto iSpace(std::make_shared<ompl::base::RealVectorStateSpace>(scene.getN() * i));
                    ompl::base::RealVectorBounds bounds(scene.getN() * i);
                    bounds.setLow(0);
                    bounds.setHigh(2*M_PI);
                    iSpace->setBounds(bounds);

                    auto iSi = std::make_shared<SpaceInformation>(iSpace);
                    iSi->setStateValidityChecker(std::make_shared<CustomValidityChecker>(iSi, scene, scene.getN(), i));

                    iSpace->setup();
                    si_vec.push_back(iSi);
                }
            }

        si_vec.push_back(si);

        auto planner = std::make_shared<T>(si_vec, name);

        return planner;
    }
    
    
    class Solver {
        Scene scene;
        AbstractionType abstractionLevel;
        ProblemDefinitionPtr pdef;
        std::vector<SpaceInformationPtr> siVec;
        bool verbose;

        public:

        Solver(Scene scene, const AbstractionType& abstractionType, bool debug) : scene(scene), abstractionLevel(abstractionType), verbose(debug) {}

        void solve(double& timeLimit) {
            if (verbose) {
                std::cout<<"------On solver::solve------"<<std::endl;
                std::cout<<"robots "<<scene.getR()<<" | joints "<<scene.getN()<<std::endl;
            }
            
            auto originSpace(std::make_shared<RealVectorStateSpace>(scene.getN() * scene.getR()));
            ompl::base::RealVectorBounds bounds(scene.getN() * scene.getR());
            ompl::geometric::SimpleSetup ss(originSpace);
            ompl::base::ScopedState<> start(originSpace), goal(originSpace);

            ompl::base::SpaceInformationPtr si = ss.getSpaceInformation();
            si->setStateValidityCheckingResolution(0.001);

            bounds.setLow(0.);
            bounds.setHigh(2*M_PI);
            originSpace->setBounds(bounds);

            ss.setStateValidityChecker(std::make_shared<mlmp::CustomValidityChecker>(si, scene, scene.getN(), scene.getR()));
    
            if(verbose) {
                std::cout<<"(*state*) start | goal"<<std::endl;
            }
            
            for (auto r=0; r<scene.getR(); ++r) {
                for (auto j=0; j<scene.getN(); ++j) {
                    if (verbose) {
                        std::cout<<"    " << scene.getStartAngleAt(r, j) << "       |      "<<scene.getGoalAngleAt(r, j)<<std::endl;
                    }
                    switch (abstractionLevel) {
                        case AbstractionType::Projection:
                            start[j*scene.getR() + r] = scene.getStartAngleAt(r, j);
                            goal[j*scene.getR() + r] = scene.getGoalAngleAt(r, j);
                            break;    
                        case AbstractionType::PriorityOrder:
                            start[r*scene.getN() + j] = scene.getStartAngleAt(r, j);
                            goal[r*scene.getN() + j] = scene.getGoalAngleAt(r, j);
                    }
                    
                }
            }
            ss.setStartAndGoalStates(start, goal);

            ompl::base::PlannerPtr planner = GetMultiLevelPlanner<ompl::multilevel::BiQRRT>(scene, abstractionLevel, si, "BiQRRT");
            ss.setPlanner(planner);


            // std::cout<<"check start"<<std::endl;
            // bool valid1 = si->isValid(start.get());

            // std::cout<<"check goal"<<std::endl;
            // bool valid2 = si->isValid(goal.get());

            bool solved = ss.solve(timeLimit);
        
            double timeToCompute = ss.getLastPlanComputationTime();
        
            if (solved)
            {
                const ompl::base::ProblemDefinitionPtr pdef = planner->getProblemDefinition();
                std::cout << std::string(80, '-') << std::endl;
                pdef->getSolutionPath()->print(std::cout);
                std::cout << std::string(80, '-') << std::endl;
                //OMPL_INFORM("Solved hypercube with %d dimensions after %f seconds.", ndim, timeToCompute);
            }
            else
            {
                OMPL_ERROR("Failed finding solution after %f seconds.", timeToCompute);
            }
        }
    };
}