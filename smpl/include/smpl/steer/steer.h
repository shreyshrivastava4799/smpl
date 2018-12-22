#ifndef SMPL_STEER_H
#define SMPL_STEER_H

#include <smpl/spatial.h>

namespace smpl {

// TODO: expose some way of looping through all (16) solutions
// TODO: order of st, sr should be consistent to avoid mistakes
// TODO: can we have different length caster axles in four-wheel steer?

namespace Wheel
{

enum Type
{
    FrontLeft = 0,
    FrontRight,
    BackLeft,
    BackRight
};

} // namespace Wheel

/// Model of a vehicle with four independently driven and steered wheels, with
/// offsets from the steering axis.
struct FourWheelSteerModel
{
    Vector2 rsv[4]; // positions of the steering frames in the vehicle frame
    double d;       // length of the caster axle
    double r;       // radius of each wheel
};

/// Model of a vehicle with two independently driven, non-steerable wheels
struct DiffSteerModel
{
    double W;   // width between each wheel
    double r;   // radius of each wheel
};

/// Model of a vehicle with four wheels, two ...TODO
struct AckermanSteerModel
{
    double L;   // distance between the front and back wheels
    double W;   // distance between the left and right wheels
    double r;   // radius of each wheel
};

// Four-wheel model state information relevant to steering kinematics
struct FourWheelSteerState
{
    double vtheta;
    double st[4];
    double sr[4];
    double vst[4];
};

struct DiffSteerState
{
    double sr[2];
};

struct AckermanSteerState
{
    double st[4];
    double sr[4];
};

// Construct a FourWheelSteerModel given the half-length and half-width between
// the two sets of wheels, symmetric around the body frame.
auto MakeFourWheelSteerModel(double HL, double HW, double d, double r)
    -> FourWheelSteerModel;

// Compute the contact velocities of each wheel wrt the steer frame
void GetContactVelocities_SF(
    const double wr,
    const double st[],
    const double sr[],
    const int count,
    Vector2 vs[]);

/////////////////////////////////
// Four-Wheel Steer Kinematics //
/////////////////////////////////

void BodyToWheel(
    const FourWheelSteerModel* model,
    const FourWheelSteerState* state,
    double vx,
    double vy,
    double vtheta,
    Vector2 vcw[4],
    double tsr[4],
    double tst[4]);

void BodyToWheelFWS(
    const Vector2 p_steer_vehicle[4],
    const double d,         // offset from steer frame to contact frame
    const double wr,        // radius of the wheels
    const double st[4],     // current steer angles
    const double vst[4],    // current rate of change of each steer angle
    const double vx,        // desired body frame x velocity
    const double vy,        // desired body frame y velocity
    const double vtheta,    // desired body frame theta velocity
    Vector2 vcw[4],         // output contact-point velocities w.r.t. the world,
                            // expressed in the vehicle frame
    double tsr[4],      // target wheel speeds
    double tst[4]);     // target steer angles

void WheelToBody(
    const FourWheelSteerModel* model,
    const FourWheelSteerState* state,
    double* vx, double* vy, double* vtheta);

void WheelToBodyFWS(
    const Vector2 p_steer_vehicle[4],
    const double d, const double wr,
    const double vt, const double st[4], const double sr[4], const double vst[4],
    double* vx, double* vy, double* vtheta);

///////////////////////////////////
// Differential Steer Kinematics //
///////////////////////////////////

void BodyToWheel(
    const DiffSteerModel* model,
    const DiffSteerState* state,
    double vx, double vy, double vtheta,
    Vector2 vcw[2], double tsr[2], double tst[2]);

void BodyToWheelDS(
    double W, double r,
    double vx, double vtheta,
    Vector2 vcw[2], double tsr[2], double tst[2]);

void WheelToBody(
    const DiffSteerModel* model,
    const DiffSteerState* state,
    double* vx,
    double* vy,
    double* vtheta);

void WheelToBodyDS(
    double W, double r,
    const double sr[2],
    double* vx, double* vy, double* vtheta);

///////////////////////////////
// Ackerman Steer Kinematics //
///////////////////////////////

void BodyToWheel(
    const AckermanSteerModel* model,
    const AckermanSteerState* state,
    double vx,
    double vtheta,
    Vector2 vcw[4],
    double tsr[4],
    double tst[4]);

void BodyToWheelAS(
    double L, double W, double r,
    const double st[4],
    double vx, double vtheta,
    Vector2 vcw[4], double tsr[4], double tst[4]);

void WheelToBody(
    const AckermanSteerModel* model,
    const AckermanSteerState* state,
    double* vx,
    double* vy,
    double* vtheta);

void WheelToBodyAS(
    double L, double W, double r,
    const double st[4], const double sr[4],
    double* vx, double* vy, double* vtheta);

} // namespace smpl

#endif

