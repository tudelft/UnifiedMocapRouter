

#include "rigid_body.hpp"
#include "pose_calculations.hpp"

#include <cmath>
#include <string>

RigidBody::RigidBody( unsigned int streaming_id,
                      ArenaDirection nose_direction ) :
        RigidBody(streaming_id, std::to_string(streaming_id), nose_direction)
{
}

RigidBody::RigidBody( unsigned int streaming_id,
                      std::string streaming_name,
                      ArenaDirection nose_direction ) :
    //velocity_filter_hz{20.f},
    //rates_filter_hz{10.f},
    streaming_id{streaming_id},
    streaming_name{streaming_name},
    unpublished_samples{0}
{
    switch(nose_direction)
    {
        // left hand pi/2 rotation around Y axis (up)
        case ArenaDirection::RIGHT: { this->nose_rot_angle = -M_PI/2.0; break; }
        // no change, because this is what we want to have
        case ArenaDirection::FAR: { this->nose_rot_angle = 0.0; break; }
        // right hand pi/2 rotation around Y axis (up)
        case ArenaDirection::LEFT: { this->nose_rot_angle = M_PI/2.0; break; }
        // pi rotation around Y
        case ArenaDirection::NEAR: { this->nose_rot_angle = M_PI; break; }
    }
    // this->poseDiff = PoseDifferentiator(); 
    this->poseDiff = FilteredPoseDifferentiator( 
        10,
        10,
        180
    );
}

pose_t RigidBody::getPoseIn( CoordinateSystem co, float north_angle )
{
    float angle = this->nose_rot_angle - north_angle;

    float nose_rot_qw = cosf(angle/2.f);
    float nose_rot_qx = 0.f;
    float nose_rot_qy = 0.f;
    float nose_rot_qz = sinf(angle/2.f);

    // perform nose rotation as quaternion rotation https://gegcalculators.com/quaternion-multiplication-calculator-online/
    // result = q_copy * nose_rot  --> use some library here? does boost have quats?
    pose_t poseNorth = this->poseENU;
    poseNorth.qw = this->poseENU.qw * nose_rot_qw - this->poseENU.qx * nose_rot_qx -  this->poseENU.qy * nose_rot_qy - this->poseENU.qz * nose_rot_qz;
    poseNorth.qx = this->poseENU.qw * nose_rot_qx + this->poseENU.qx * nose_rot_qw +  this->poseENU.qy * nose_rot_qz - this->poseENU.qz * nose_rot_qy;
    poseNorth.qy = this->poseENU.qw * nose_rot_qy - this->poseENU.qx * nose_rot_qz +  this->poseENU.qy * nose_rot_qw + this->poseENU.qz * nose_rot_qx;
    poseNorth.qz = this->poseENU.qw * nose_rot_qz + this->poseENU.qx * nose_rot_qy -  this->poseENU.qy * nose_rot_qx + this->poseENU.qz * nose_rot_qw;

    pose_t poseCo = poseNorth;
    switch(co)
    {
        case CoordinateSystem::ENU:
            // Transform to ENU
            // do nothing, because thats what we want
            poseCo.x = +cos(north_angle) * poseNorth.x + sin(north_angle) * poseNorth.y;
            poseCo.y = -sin(north_angle) * poseNorth.x + cos(north_angle) * poseNorth.y;
            poseCo.z = poseNorth.z;

            poseCo.qx = +cos(north_angle) * poseNorth.qx + sin(north_angle) * poseNorth.qy;
            poseCo.qy = -sin(north_angle) * poseNorth.qx + cos(north_angle) * poseNorth.qy;
            poseCo.qz = poseNorth.qz;
            break;
        case CoordinateSystem::NED:
            // Transform to NED
            poseCo.x = +cos(north_angle) * poseNorth.y - sin(north_angle) * poseNorth.x;
            poseCo.y = +sin(north_angle) * poseNorth.y + cos(north_angle) * poseNorth.x;
            poseCo.z = -poseNorth.z;

            poseCo.qx = +cos(north_angle) * poseNorth.qy - sin(north_angle) * poseNorth.qx;
            poseCo.qy = +sin(north_angle) * poseNorth.qy + cos(north_angle) * poseNorth.qx;
            poseCo.qz = -poseNorth.qz;
            break;
        default:
            break;
    }

    return poseCo;
}

twist_t RigidBody::getTwistIn( CoordinateSystem co, float north_angle )
{
    float angle = this->nose_rot_angle - north_angle;

    float nose_rot_qw = cosf(angle/2.f);
    float nose_rot_qx = 0.f;
    float nose_rot_qy = 0.f;
    float nose_rot_qz = sinf(angle/2.f);

    // dont know how to do this for body rates
    twist_t twistNorth = this->twistENU;
    twistNorth.wx = 0.f;
    twistNorth.wy = 0.f;
    twistNorth.wz = 0.f;
    //twistNorth.qw = this->twistENU.qw * nose_rot_qw - this->twistENU.qx * nose_rot_qx -  this->twistENU.qy * nose_rot_qy - this->twistENU.qz * nose_rot_qz;
    //twistNorth.qx = this->twistENU.qw * nose_rot_qx + this->twistENU.qx * nose_rot_qw +  this->twistENU.qy * nose_rot_qz - this->twistENU.qz * nose_rot_qy;
    //twistNorth.qy = this->twistENU.qw * nose_rot_qy - this->twistENU.qx * nose_rot_qz +  this->twistENU.qy * nose_rot_qw + this->twistENU.qz * nose_rot_qx;
    //twistNorth.qz = this->twistENU.qw * nose_rot_qz + this->twistENU.qx * nose_rot_qy -  this->twistENU.qy * nose_rot_qx + this->twistENU.qz * nose_rot_qw;

    twist_t twistCo = twistNorth;
    switch(co)
    {
        case CoordinateSystem::ENU:
            // Transform to ENU
            // do nothing, because thats what we want
            twistCo.vx = +cos(north_angle) * twistNorth.vx + sin(north_angle) * twistNorth.vy;
            twistCo.vy = -sin(north_angle) * twistNorth.vx + cos(north_angle) * twistNorth.vy;
            twistCo.vz = twistNorth.vz;

            //twistCo.qx = +cos(north_angle) * twistNorth.qx + sin(north_angle) * twistNorth.qy;
            //twistCo.qy = -sin(north_angle) * twistNorth.qx + cos(north_angle) * twistNorth.qy;
            //twistCo.qz = twistNorth.qz;
            break;
        case CoordinateSystem::NED:
            // Transform to NED
            twistCo.vx = +cos(north_angle) * twistNorth.vy - sin(north_angle) * twistNorth.vx;
            twistCo.vy = +sin(north_angle) * twistNorth.vy + cos(north_angle) * twistNorth.vx;
            twistCo.vz = -twistNorth.vz;

            //twistCo.qx = +cos(north_angle) * twistNorth.qy - sin(north_angle) * twistNorth.qx;
            //twistCo.qy = +sin(north_angle) * twistNorth.qy + cos(north_angle) * twistNorth.qx;
            //twistCo.qz = -twistNorth.qz;
            break;
        default:
            break;
    }

    return twistCo;
}

void RigidBody::setNewPoseENU_NorthFarSide(pose_t& newPose)
{
    this->poseENU = newPose;
    this->twistENU = this->poseDiff.apply(newPose);
    this->unpublished_samples++;
    this->latest_sample_time = newPose.timeUs;
}
