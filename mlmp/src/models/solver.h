#include <math.h>
#include <boost/range/algorithm_ext/push_back.hpp>

#include <abstraction_type.h>
#include <scene.h>
#include <state/kinematic_chain.h>
#include "common/segment.h"


#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateSpace.h>
#include <ompl/multilevel/planners/qrrt/QRRT.h>

typedef mlmp::abstraction::AbstractionType AbstractionType;
using namespace ompl::base;
std::vector<Environment> envs;

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

    PlannerPtr GetPlanner(std::vector<int> sequenceLinks, double linkLength, int numLinks, SpaceInformationPtr si)
    {
        std::vector<SpaceInformationPtr> si_vec;
    
        std::cout<<sequenceLinks.size()<<std::endl;
        for (unsigned int k = 0; k < sequenceLinks.size(); k++)
        {
            auto links = sequenceLinks.at(k) - 1;
            std::cout<<links<<std::endl;
            assert(links < numLinks);
            OMPL_INFORM("Create MultiLevel Chain with %d links.", links);
            auto spaceK(std::make_shared<KinematicChainSpace>(links, linkLength, &envs.at(links)));

            auto siK = std::make_shared<SpaceInformation>(spaceK);
            siK->setStateValidityChecker(std::make_shared<KinematicChainValidityChecker>(siK));
            spaceK->setup();
            si_vec.push_back(siK);
        }
    
        OMPL_INFORM("Add Original Chain with %d links.", numLinks);
        si_vec.push_back(si);
    
        auto planner = std::make_shared<ompl::multilevel::BiQRRT>(si_vec);
    
        std::string qName = "BiQRRT[";
        for (unsigned int k = 0; k < sequenceLinks.size(); k++)
        {
            int links = sequenceLinks.at(k);
            qName += std::to_string(links) + ",";
        }
        qName += std::to_string(numLinks);
        qName += "]";
        planner->setName(qName);
        return planner;
    }
    

    std::vector<std::vector<int>> getAdmissibleProjections(int dim)
    {
        std::vector<std::vector<int>> projections;
        std::vector<int> discrete;
        boost::push_back(discrete, boost::irange(2, dim + 1));

        std::vector<int> twoStep;
        boost::push_back(twoStep, boost::irange(2, dim + 1, 2));

        if (twoStep.back() != dim)
            twoStep.push_back(dim);

        projections.push_back(twoStep);
        projections.push_back(discrete);

        auto last = std::unique(projections.begin(), projections.end());
        projections.erase(last, projections.end());

        std::cout << "Projections for dim " << dim << std::endl;
        for (unsigned int k = 0; k < projections.size(); k++)
        {
            std::vector<int> pk = projections.at(k);
            std::cout << k << ": ";
            for (unsigned int j = 0; j < pk.size(); j++)
            {
                std::cout << pk.at(j) << (j < pk.size() - 1 ? "," : "");
            }
            std::cout << std::endl;
        }

        return projections;
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

            std::vector<mlmp::common::Segment> commonSegments = scene.getObstaclesSegments();
            Environment env = commonSegments;
    
            for (unsigned int k = 0; k < scene.getN(); k++)
            {
                envs.push_back(scene.getObstaclesSegments());
            }

            auto chain(std::make_shared<KinematicChainSpace>(scene.getN(), scene.getJointLength(), &env));
            ompl::geometric::SimpleSetup ss(chain);
        
            ss.setStateValidityChecker(std::make_shared<KinematicChainValidityChecker>(ss.getSpaceInformation()));
        
            ompl::base::ScopedState<> start(chain), goal(chain);
            std::vector<double> startVec = scene.getStartAngles()[0];
            std::vector<double> goalVec = scene.getGoalAngles()[0];
            chain->setup();
            chain->copyFromReals(start.get(), startVec);
            chain->copyFromReals(goal.get(), goalVec);
            ss.setStartAndGoalStates(start, goal);
            
            // std::vector<std::vector<int>> admissibleProjections = getAdmissibleProjections(scene.getN() - 1);
            std::vector<int> discrete;
            boost::push_back(discrete, boost::irange(2, scene.getN() + 1));
            ompl::base::PlannerPtr planner = GetPlanner(discrete, scene.getJointLength(), scene.getN(), ss.getSpaceInformation());

            ss.setPlanner(planner);
            
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