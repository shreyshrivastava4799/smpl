#ifndef SMPL_STEER_H
#define SMPL_STEER_H

#include <smpl/spatial.h>

namespace smpl {

struct VehicleModel
{
    int num_wheels;
};

struct FourSteerModel
{
    VehicleModel model;

    // x distance from the vehicle frame to each steer frame
    double L;

    // y distance from the vehicle frame to each steer frame
    double H;

    // length of the caster axle
    double d;

    // radius of each wheel
    double r;

    // TODO: define order and derive me from L, H
    // positions of the steering frames in the vehicle frame
    Vector2 rsv[4];
};

struct DiffSteerModel
{
    VehicleModel model;

    double W;   // width between each wheel
    double r;   // radius of each wheel
};

struct AckermanSteerModel
{
    VehicleModel model;

    double L;
    double W;
    double r;   // radius of each wheel
};

struct VehicleState
{
    // pose of the body frame in the world frame
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;

    // velocity of the body frame in the body frame
    double vx = 0.0;
    double vy = 0.0;
    double vtheta = 0.0;

    double* ast = NULL;     // array of steer angles
    double* asr = NULL;     // array of wheel positions
    double* avst = NULL;    // array of steer angle velocities
    double* avsr = NULL;    // array of wheel velocities
};

struct FourSteerState
{
    VehicleState state;

    double st[4] = { 0 };
    double sr[4] = { 0 };
    double vst[4] = { 0 };
    double vsr[4] = { 0 };
};

struct DiffSteerState
{
    VehicleState state;

    double st[2] = { 0 };
    double sr[2] = { 0 };

    double vst[2] = { 0 };
    double vsr[2] = { 0 };
};

struct AckermanSteerState
{
    VehicleState state;

    double st[4] = { 0 };
    double sr[4] = { 0 };

    double vst[4] = { 0 };
    double vsr[4] = { 0 };
};

// Get the positions of the wheel contact points in their respect steer frames,
// assuming all 4 wheels are the same distance d away from the steer frames.
void GetContactPositions_SF(double d, const double st[4], Vector2 r_cs[4]);

void GetContactVelocities_SF(
    const double wr,
    const double st[],
    const double sr[],
    const int count,
    Vector2 vs[]);

// Compute the contact velocities of each wheel wrt the steer frame
void GetContactVelocities_SF(
    const double wr,
    const double st[4],
    const double sr[4],
    Vector2 v_cs[4]);

void ComputeWheelFK(
    const FourSteerModel* model,
    const FourSteerState* state,
    double vx,
    double vy,
    double vtheta,
    Vector2 vcw[4],
    double tsr[4],
    double tst[4]);

// TODO: is this overload useful?
void ComputeWheelFK(
    const Vector2 steer_positions_vf[4],
    const double d,         // offset from steer frame to contact frame
    const double wr,        // radius of the wheels
    const double st[4],     // current steer angles
    const double sr[4],     // current wheel speeds
    const double vst[4],    // current rate of change of each steer angle
    const double vsr[4],
    const double vx,        // desired body frame x velocity
    const double vy,        // desired body frame y velocity
    const double vtheta,    // desired body frame theta velocity
    Vector2 vcw[4],           // output contact velocities wrt the world in the vehicle frame
    double tsr[4],          // target wheel speeds
    double tst[4]);         // target steer angles

void ComputeWheelIK(
    const FourSteerModel* model,
    const FourSteerState* state,
    double* vx,
    double* vy,
    double* vtheta);

void ComputeWheelIK(
    const Vector2 steer_positions_vf[4],
    const double d,
    const double wr,
    const double vt,
    const double st[4],
    const double sr[4],
    const double vst[4],
    const double vsr[4],
    double* vx,
    double* vy,
    double* vtheta);

void ComputeWheelFK(
    const DiffSteerModel* model,
    const DiffSteerState* state,
    double vx,
    double vy,
    double vtheta,
    Vector2 vcw[2],
    double tsr[2],
    double tst[2]);

void ComputeWheelIK(
    const DiffSteerModel* model,
    const DiffSteerState* state,
    double* vx,
    double* vy,
    double* vtheta);

void ComputeWheelFK(
    const AckermanSteerModel* model,
    const AckermanSteerState* state,
    double vx,
    double vy,
    double vtheta,
    Vector2 vcw[4],
    double tsr[4],
    double tst[4]);

void ComputeWheelIK(
    const AckermanSteerModel* model,
    const AckermanSteerState* state,
    double* vx,
    double* vy,
    double* vtheta);

} // namespace smpl

#endif

