// Copyright 2024 Anton Bredenbeck, Till Blaha (Delft University of Technology)
//
// This program is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the Free
// Software Foundation, either version 3 of the License, or (at your option)
// any later version.
//
// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
// FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
// more details.
//
// You should have received a copy of the GNU General Public License along
// with this program. If not, see <https://www.gnu.org/licenses/>.

#include "pose_calculations.hpp"

std::ostream& operator<<(std::ostream& lhs, CoordinateSystem e) {
    switch(e) {
    //case UNCHANGED: lhs << "UNCHANGED"; break;
    case NED: lhs << "NED"; break;
    case ENU: lhs << "ENU"; break;
    }
    return lhs;
} 

std::ostream& operator<<(std::ostream& lhs, ArenaDirection e) {
    switch(e) {
    case RIGHT: lhs << "RIGHT"; break;
    case FAR: lhs << "FAR"; break;
    case LEFT: lhs << "LEFT"; break;
    case NEAR: lhs << "NEAR"; break;
    }
    return lhs;
} 

void quaternion_of_rotationMatrix(quaternion_t *q, const rotationMatrix_t *r) {
    // https://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
    float trace = r->m[0][0] + r->m[1][1] + r->m[2][2];
    float s, si;
    if (trace > 1e-6f) {
        s = 0.5f * sqrtf( 1.0f + trace );
        si = 0.25f / s;
        q->w = s;
        q->x = si * ( r->m[2][1] - r->m[1][2] );
        q->y = si * ( r->m[0][2] - r->m[2][0] );
        q->z = si * ( r->m[1][0] - r->m[0][1] );
    } else {
        if ( r->m[0][0] > r->m[1][1] && r->m[0][0] > r->m[2][2] ) {
            s = 0.5f * sqrtf( 1.0f + r->m[0][0] - r->m[1][1] - r->m[2][2]);
            si = 0.25f / s;
            q->w = si * (r->m[2][1] - r->m[1][2] );
            q->x = s;
            q->y = si * (r->m[0][1] + r->m[1][0] );
            q->z = si * (r->m[0][2] + r->m[2][0] );
        } else if (r->m[1][1] > r->m[2][2]) {
            s = 0.5f * sqrtf( 1.0f + r->m[1][1] - r->m[0][0] - r->m[2][2]);
            si = 0.25f / s;
            q->w = si * (r->m[0][2] - r->m[2][0] );
            q->x = si * (r->m[0][1] + r->m[1][0] );
            q->y = s;
            q->z = si * (r->m[1][2] + r->m[2][1] );
        } else {
            s = 0.5f * sqrtf( 1.0f + r->m[2][2] - r->m[0][0] - r->m[1][1] );
            si = 0.25f / s;
            q->w = si * (r->m[1][0] - r->m[0][1] );
            q->x = si * (r->m[0][2] + r->m[2][0] );
            q->y = si * (r->m[1][2] + r->m[2][1] );
            q->z = s;
        }
    }
}

void PoseDifferentiator::newSample(pose_t newPose)
{
    int64_t delta = newPose.timeUs - _pose.timeUs; // does this deal with overflows?

    if ((delta < 100) || (delta > 2e6) || (_pose.timeUs == 0)) {
        // likely uninitialized, or very old data
        _pose = newPose;
        _valid = false;
        return;
    }
    _valid = true;

    float iDelta = 1.0 / (static_cast<float>(delta) * 1e-6f);
    _unfiltered.timeUs = newPose.timeUs;
    _unfiltered.vx = iDelta * (newPose.x - _pose.x);
    _unfiltered.vy = iDelta * (newPose.y - _pose.y);
    _unfiltered.vz = iDelta * (newPose.z - _pose.z);

    // https://mariogc.com/post/angular-velocity-quaternions/
    float q1[4] = {_pose.qw, _pose.qx, _pose.qy, _pose.qz};
    float q2[4] = {newPose.qw, newPose.qx, newPose.qy, newPose.qz};
    _unfiltered.wx = 2.*iDelta * (q1[0]*q2[1] - q1[1]*q2[0] - q1[2]*q2[3] + q1[3]*q2[2]);
    _unfiltered.wy = 2.*iDelta * (q1[0]*q2[2] + q1[1]*q2[3] - q1[2]*q2[0] - q1[3]*q2[1]);
    _unfiltered.wz = 2.*iDelta * (q1[0]*q2[3] - q1[1]*q2[2] + q1[2]*q2[1] - q1[3]*q2[0]);

    _pose = newPose;
}

FilteredPoseDifferentiator::FilteredPoseDifferentiator(float fBreakVel,
                                                       float fBreakRate,
                                                       float fSample)
    : PoseDifferentiator(), _initialized{false}, _filtered{0,
                                                           0, 0, 0,
                                                           0, 0, 0}
{
    _fSample = fSample;
    _fBreakVel = fBreakVel;
    _fBreakRate = fBreakRate;
    if (fBreakVel < 1e-3) {
        _kRate = 0.;
        std::cout << "WARNING: Velocity lowpass filter cannot be realized and will be disabled. Check fBreakVel." << std::endl;
        return;
    }
    if (fBreakRate < 1e-3) {
        _kVel = 0.;
        std::cout << "WARNING: Angular rate lowpass filter cannot be realized and will be disabled. Check fBreakRate." << std::endl;
        return;
    }

    // https://en.wikipedia.org/wiki/Low-pass_filter#Simple_infinite_impulse_response_filter
    _kVel = 1. / (1. + _fSample / (2. * M_PI * _fBreakVel) );
    _kRate = 1. / (1. + _fSample / (2. * M_PI * _fBreakRate) );
}

twist_t FilteredPoseDifferentiator::apply(pose_t newPose)
{
    newSample(newPose);
    if (!_valid)
        return _filtered;

    if (_initialized) {
        _filtered.vx = _kVel * _filtered.vx  +  (1.-_kVel) * _unfiltered.vx; 
        _filtered.vy = _kVel * _filtered.vy  +  (1.-_kVel) * _unfiltered.vy; 
        _filtered.vz = _kVel * _filtered.vz  +  (1.-_kVel) * _unfiltered.vz; 
        _filtered.wx = _kRate * _filtered.wx  +  (1.-_kRate) * _unfiltered.wx;
        _filtered.wy = _kRate * _filtered.wy  +  (1.-_kRate) * _unfiltered.wy; 
        _filtered.wz = _kRate * _filtered.wz  +  (1.-_kRate) * _unfiltered.wz; 
    } else {
        _filtered = _unfiltered;
        _initialized = true;
    }

    return _filtered;
}
