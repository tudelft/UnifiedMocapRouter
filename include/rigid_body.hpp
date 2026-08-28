
#ifndef H_RIGID_BODY
#define H_RIGID_BODY

#include "pose_calculations.hpp"

class RigidBody
{
    private:
        pose_t poseENU;
        twist_t twistENU;
        float nose_rot_angle;
        unsigned int unpublished_samples;

        uint64_t latest_sample_time;
        uint64_t last_published_sample_time;

        //float velocity_filter_hz; // not implemented yet
        //float rates_filter_hz;
        //FilteredPoseDifferentiator poseDiff;
        PoseDifferentiator poseDiff;

    public:
        unsigned int streaming_id;
        std::string streaming_name;

        RigidBody(unsigned int streaming_id,
                  ArenaDirection nose_direction);
        RigidBody(unsigned int streaming_id,
                  std::string streaming_name,
                  ArenaDirection nose_direction);

        void enableDerivativeFiltering(float fBreakVel, float fBreakRate, float fSample);
        pose_t getPoseIn( CoordinateSystem co, float north_angle );
        twist_t getTwistIn( CoordinateSystem co, float north_angle );
        void setNewPoseENU_NorthFarSide(pose_t& newPose);
        void setPublished() { 
            this->unpublished_samples = 0;
            this->last_published_sample_time = this->poseENU.timeUs;
        };
        unsigned int getNumUnpublishedSamples() { return this->unpublished_samples; };
        uint64_t getLatestSampleTime() { return this->latest_sample_time; };
        uint64_t getLastPublishedSampleTime() { return this->last_published_sample_time; };
};

#endif