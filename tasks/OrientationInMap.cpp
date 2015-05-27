/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "OrientationInMap.hpp"

using namespace orientation_estimator;

OrientationInMap::OrientationInMap(std::string const& name)
    : OrientationInMapBase(name)
{
}

OrientationInMap::OrientationInMap(std::string const& name, RTT::ExecutionEngine* engine)
    : OrientationInMapBase(name, engine)
{
}

OrientationInMap::~OrientationInMap()
{
}

void OrientationInMap::orientation_in_worldTransformerCallback(const base::Time &ts, const ::base::samples::RigidBodyState &orientation_in_world_sample)
{
    // receive map to world transformation
    Eigen::Affine3d map2world;
    if (!_map2world.get(ts, map2world))
    {
        RTT::log(RTT::Error) << "skip, have no map2world transformation sample!" << RTT::endlog();
        new_state = MISSING_TRANSFORMATION;
        return;
    }
    
    // write orientation in map sample
    base::samples::RigidBodyState orientation_in_map = orientation_in_world_sample;
    orientation_in_map.targetFrame = _map_frame.value();
    orientation_in_map.setTransform(map2world.inverse() * orientation_in_world_sample.getTransform());
    _orientation_in_map.write(orientation_in_map);
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See OrientationInMap.hpp for more detailed
// documentation about them.

bool OrientationInMap::configureHook()
{
    if (! OrientationInMapBase::configureHook())
        return false;
    
    last_state = PRE_OPERATIONAL;
    new_state = RUNNING;
    
    return true;
}
bool OrientationInMap::startHook()
{
    if (! OrientationInMapBase::startHook())
        return false;
    return true;
}
void OrientationInMap::updateHook()
{
    new_state = RUNNING;
    
    OrientationInMapBase::updateHook();
    
    // write task state if it has changed
    if(last_state != new_state)
    {
        last_state = new_state;
        state(new_state);
    }
}
void OrientationInMap::errorHook()
{
    OrientationInMapBase::errorHook();
}
void OrientationInMap::stopHook()
{
    OrientationInMapBase::stopHook();
}
void OrientationInMap::cleanupHook()
{
    OrientationInMapBase::cleanupHook();
}
