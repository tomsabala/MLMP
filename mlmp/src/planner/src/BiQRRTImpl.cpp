#include <../BiQRRTImpl.h>
#include <../tools/SelfConfig.h>

#include <ompl/multilevel/datastructures/graphsampler/GraphSampler.h>
#include <ompl/multilevel/datastructures/propagators/Geometric.h>
#include <ompl/multilevel/datastructures/metrics/Geodesic.h>
#include <boost/foreach.hpp>

#define foreach BOOST_FOREACH
using namespace ompl::multilevel;
 
ompl::multilevel::BiQRRTImpl::BiQRRTImpl(const base::SpaceInformationPtr &si, BundleSpace *parent_) : BaseT(si, parent_)
{
    setName("BiQRRTImpl" + std::to_string(id_));
    setImportance("exponential");
    setGraphSampler("randomvertex");
    getGraphSampler()->disableSegmentBias();
    distanceBetweenTrees_ = std::numeric_limits<double>::infinity();
}

ompl::multilevel::BiQRRTImpl::~BiQRRTImpl(){}

void BiQRRTImpl::setup()
{
    BaseT::setup();

    maxDistance_ = 0.1;

    if (!treeStart_)
    {
        treeStart_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Configuration *>(this));
        treeStart_->setDistanceFunction(
            [this](const Configuration *a, const Configuration *b) { return distance(a, b); });
    }
    if (!treeGoal_)
    {
        treeGoal_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Configuration *>(this));
        treeGoal_->setDistanceFunction(
            [this](const Configuration *a, const Configuration *b) { return distance(a, b); });
    }
}
void BiQRRTImpl::clear()
{
    BaseT::clear();
    if (treeStart_)
        treeStart_->clear();
    if (treeGoal_)
        treeGoal_->clear();
    distanceBetweenTrees_ = std::numeric_limits<double>::infinity();
}

void BiQRRTImpl::init()
{
    if (const base::State *state = pis_.nextStart())
    {
        qStart_ = new Configuration(getBundle(), state);
        qStart_->isStart = true;
        Vertex m = boost::add_vertex(qStart_, graph_);
        disjointSets_.make_set(m);
        qStart_->index = m;
        treeStart_->add(qStart_);
    }

    if (qStart_ == nullptr)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        throw ompl::Exception("Invalid initial states.");
    }

    if (const base::State *state = pis_.nextGoal())
    {
        qGoal_ = new Configuration(getBundle(), state);
        qGoal_->isGoal = true;
        Vertex m = boost::add_vertex(qGoal_, graph_);
        disjointSets_.make_set(m);
        qGoal_->index = m;
        treeGoal_->add(qGoal_);
        goalConfigurations_.push_back(qGoal_);
    }

    if (qGoal_ == nullptr && getGoalPtr()->canSample())
    {
        OMPL_ERROR("%s: There are no valid goal states!", getName().c_str());
        throw ompl::Exception("Invalid goal states.");
    }
}

void BiQRRTImpl::grow()
{
    //(0) If first run, add start configuration
    if (firstRun_)
    {
        init();
        firstRun_ = false;

        findSection();
    }

    TreeData &tree = activeInitialTree_ ? treeStart_ : treeGoal_;
    activeInitialTree_ = !activeInitialTree_;
    TreeData &otherTree = activeInitialTree_ ? treeStart_ : treeGoal_;

    //###########################################################
    //(1) Get Uniform Random Sample
    sampleBundle(xRandom_->state);

    //###########################################################
    //(2) Get Nearest in Current Highlighted Tree
    const Configuration *xNearest = tree->nearest(xRandom_);

    //###########################################################
    //(3) Connect Nearest to Random (within range)
    // Configuration *xNext = extendGraphTowards_Range(xNearest, xRandom_);

    double d = distance(xNearest, xRandom_);
    if (d > maxDistance_)
    {
        metric_->interpolateBundle(xNearest, xRandom_, maxDistance_ / d, xRandom_);
    }

    if (!propagator_->steer(xNearest, xRandom_, xRandom_))
    {
        return;
    }

    Configuration *xNext = new Configuration(getBundle(), xRandom_->state);
    Vertex m = boost::add_vertex(xNext, graph_);
    disjointSets_.make_set(m);
    xNext->index = m;
    tree->add(xNext);
    addBundleEdge(xNearest, xNext);

    //###########################################################
    //(4) If extension was successful, check if we reached goal
    if (xNext && !hasSolution_)
    {
        /* update distance between trees */
        Configuration *xOtherTree = otherTree->nearest(xNext);
        const double newDist = tree->getDistanceFunction()(xNext, xOtherTree);
        if (newDist < distanceBetweenTrees_)
        {
            distanceBetweenTrees_ = newDist;
            OMPL_INFORM("Estimated distance to go: %f", distanceBetweenTrees_);
        }

        bool satisfied = propagator_->steer(xNext, xOtherTree, xRandom_);

        if (satisfied)
        {
            addBundleEdge(xNext, xOtherTree);
            hasSolution_ = true;
        }
    }
}

void BiQRRTImpl::getPlannerData(ompl::base::PlannerData &data) const
{
    BaseT::getPlannerData(data);
    OMPL_DEBUG(" Start Tree has %d vertices, Goal Tree has %d vertices.", treeStart_->size(), treeGoal_->size());
}