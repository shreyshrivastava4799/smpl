#include <smpl/steer/steer.h>

#include <cmath>

namespace smpl {

// Get the positions of the wheel contact points in their respect steer frames,
// assuming all 4 wheels are the same distance d away from the steer frames.
void GetContactPositions_SF(double d, const double st[4], Vector2 r_cs[4])
{
    // modified from textbook definition
    r_cs[0] = d * Vector2(-std::sin(st[0]),  std::cos(st[0]));
    r_cs[1] = d * Vector2( std::sin(st[1]), -std::cos(st[1]));
    r_cs[2] = d * Vector2(-std::sin(st[2]),  std::cos(st[2]));
    r_cs[3] = d * Vector2( std::sin(st[3]), -std::cos(st[3]));
}

// Extract the x coordinates from a set of points.
static void get_xs(const Vector2* points, double* xs, int count)
{
    for (auto i = 0; i < count; ++i) {
        xs[i] = points[i].x();
    }
}

// Extract the y coordinates from a set of points.
static void get_ys(const Vector2* points, double* ys, int count)
{
    for (auto i = 0; i < count; ++i) {
        ys[i] = points[i].y();
    }
}

static auto Compute_H_c(const Vector2 r_cs[4]) -> Matrix<double, 8, 5>
{
    double a[4];
    get_xs(r_cs, a, 4);

    double b[4];
    get_ys(r_cs, b, 4);

    Matrix<double, 8, 5> H_c;

    // textbook definition
    H_c(0,0) = -b[0]; H_c(0,1) = -b[0]; H_c(0,2) =     0; H_c(0,3) =     0; H_c(0,4) =     0;
    H_c(1,0) =  a[0]; H_c(1,1) =  a[0]; H_c(1,2) =     0; H_c(1,3) =     0; H_c(1,4) =     0;
    H_c(2,0) = -b[1]; H_c(2,1) =     0; H_c(2,2) = -b[1]; H_c(2,3) =     0; H_c(2,4) =     0;
    H_c(3,0) =  a[1]; H_c(3,1) =     0; H_c(3,2) =  a[1]; H_c(3,3) =     0; H_c(3,4) =     0;
    H_c(4,0) = -b[2]; H_c(4,1) =     0; H_c(4,2) =     0; H_c(4,3) = -b[2]; H_c(4,4) =     0;
    H_c(5,0) =  a[2]; H_c(5,1) =     0; H_c(5,2) =     0; H_c(5,3) =  a[2]; H_c(5,4) =     0;
    H_c(6,0) = -b[3]; H_c(6,1) =     0; H_c(6,2) =     0; H_c(6,3) =     0; H_c(6,4) = -b[3];
    H_c(7,0) =  a[3]; H_c(7,1) =     0; H_c(7,2) =     0; H_c(7,3) =     0; H_c(7,4) =  a[3];

    return H_c;
}

static auto Compute_H_s(const Vector2 steer_positions_vf[4]) -> Matrix<double, 8, 3>
{
    double x[4];
    get_xs(steer_positions_vf, x, 4);

    double y[4];
    get_ys(steer_positions_vf, y, 4);

    Matrix<double, 8, 3> H_s;

    H_s(0,0) = 1; H_s(0,1) = 0; H_s(0,2) = -y[0];
    H_s(1,0) = 0; H_s(1,1) = 1; H_s(1,2) =  x[0];
    H_s(2,0) = 1; H_s(2,1) = 0; H_s(2,2) = -y[1];
    H_s(3,0) = 0; H_s(3,1) = 1; H_s(3,2) =  x[1];
    H_s(4,0) = 1; H_s(4,1) = 0; H_s(4,2) = -y[2];
    H_s(5,0) = 0; H_s(5,1) = 1; H_s(5,2) =  x[2];
    H_s(6,0) = 1; H_s(6,1) = 0; H_s(6,2) = -y[3];
    H_s(7,0) = 0; H_s(7,1) = 1; H_s(7,2) =  x[3];

    return H_s;
}

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
    double tst[4])          // target steer angles
{
    if ((vx == 0.0) & (vy == 0.0) & (vtheta == 0.0)) {
        vcw[0] = vcw[1] = vcw[2] = vcw[3] = Vector2::Zero();
        tsr[0] = tsr[1] = tsr[2] = tsr[3] = 0.0;
        tst[0] = st[0];
        tst[1] = st[1];
        tst[2] = st[2];
        tst[3] = st[3];
        return;
    }
    auto H_s = Compute_H_s(steer_positions_vf);

    auto V_s = Matrix<double, 3, 1>(vx, vy, vtheta);

    Matrix<double, 8, 1> v;

    // target contact velocities wrt the world, in the vehicle frame
    // 8x1 = 8x3 * 3x1;
    // NOTE: compute steer rates before determining steer angles
    v = H_s * V_s; // + H_c * V_c;

    // v -> consistent steer angles
    tst[0] = atan2(v[1], v[0]);
    tst[1] = atan2(v[3], v[2]);
    tst[2] = atan2(v[5], v[4]);
    tst[3] = atan2(v[7], v[6]);

    // target steer angles -> contact_positions[steer]
    Vector2 contact_positions_SF[4];
    GetContactPositions_SF(d, tst, contact_positions_SF);

    // contact_positions[steer] -> H_c
    auto H_c = Compute_H_c(contact_positions_SF);

    Matrix<double, 5, 1> V_c;
    V_c(0) = vtheta;
    V_c(1) = vst[0];
    V_c(2) = vst[1];
    V_c(3) = vst[2];
    V_c(4) = vst[3];

    // update target contact velocities, wrt the world, in the vehicle frame
    // using the contributions from vtheta and vst at the target steer angles
    // 8x1 + 8x5 * 5x1
    v += H_c * V_c; // = H_s * V_s + H_c * V_c
    vcw[0] = Vector2(v[0], v[1]);
    vcw[1] = Vector2(v[2], v[3]);
    vcw[2] = Vector2(v[4], v[5]);
    vcw[3] = Vector2(v[6], v[7]);

    // target contact velocities -> target wheel speeds
    tsr[0] = vcw[0].norm() / wr;
    tsr[1] = vcw[1].norm() / wr;
    tsr[2] = vcw[2].norm() / wr;
    tsr[3] = vcw[3].norm() / wr;

    // 16 possible solutions, take the one with the closest steer angles
    for (auto i = 0; i < 4; ++i) {
        if (smpl::shortest_angle_dist(st[i], tst[i]) > 0.5 * M_PI) {
            tst[i] += M_PI;
            tsr[i] *= -1.0;
        }
    }
}

void GetContactVelocities_SF(
    const double wr,
    const double st[],
    const double sr[],
    const int count,
    Vector2 vs[])
{
    for (auto i = 0; i < count; ++i) {
        vs[i] = sr[i] * wr * Vector2(std::cos(st[i]), std::sin(st[i]));
    }
}

// Compute the contact velocities of each wheel wrt the steer frame
void GetContactVelocities_SF(
    const double wr,
    const double st[4],
    const double sr[4],
    Vector2 v_cs[4])
{
    v_cs[0] = sr[0] * wr * Vector2(std::cos(st[0]), std::sin(st[0]));
    v_cs[1] = sr[1] * wr * Vector2(std::cos(st[1]), std::sin(st[1]));
    v_cs[2] = sr[2] * wr * Vector2(std::cos(st[2]), std::sin(st[2]));
    v_cs[3] = sr[3] * wr * Vector2(std::cos(st[3]), std::sin(st[3]));
}

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
    double* vtheta)
{
    Vector2 v_cs[4]; // velocities of each contact
    GetContactVelocities_SF(wr, st, sr, v_cs);

    // flatten r_cs to 8x1 matrix
    Matrix<double, 8, 1> v_c;
    v_c(0) = v_cs[0].x();
    v_c(1) = v_cs[0].y();
    v_c(2) = v_cs[1].x();
    v_c(3) = v_cs[1].y();
    v_c(4) = v_cs[2].x();
    v_c(5) = v_cs[2].y();
    v_c(6) = v_cs[3].x();
    v_c(7) = v_cs[3].y();

    Matrix<double, 5, 1> V_c;
    V_c(0) = vt;
    V_c(1) = vst[0]; //0.0; //sr[0];
    V_c(2) = vst[1]; //0.0; //sr[1];
    V_c(3) = vst[2]; //0.0; //sr[2];
    V_c(4) = vst[3]; //0.0; //sr[3];

    // positions of the contact frames, in their respective steer frames
    Vector2 r_cs[4];
    GetContactPositions_SF(d, st, r_cs);

    auto H_c = Compute_H_c(r_cs);

    Matrix<double, 8, 1> v_s = v_c - H_c * V_c;

    auto H_s = Compute_H_s(steer_positions_vf);

    // (3x8 * 8x3)^1 * 3x8 * 8x1 = 3x3
    Matrix<double, 3, 1> V_s = (H_s.transpose() * H_s).inverse() * H_s.transpose() * v_s;
    *vx = V_s(0);
    *vy = V_s(1);
    *vtheta = V_s(2);
}

void ComputeWheelFK(
    const DiffSteerModel* model,
    const DiffSteerState* state,
    double vx,
    double vy,
    double vtheta,
    Vector2 vcw[2],
    double tsr[2],
    double tst[2])
{
    auto M = Matrix2();
    M(0,0) = 1.0; M(0,1) = model->W;
    M(1,0) = 1.0; M(1,1) = -model->W;

    auto v = Vector2(M * Vector2(vx, vtheta));

    vcw[0] = Vector2(v(1), 0.0);
    vcw[1] = Vector2(v(0), 0.0);

    tsr[1] = v(0) / model->r;
    tsr[0] = v(1) / model->r;

    tst[0] = 0.0;
    tst[1] = 0.0;
}

void ComputeWheelIK(
    const DiffSteerModel* model,
    const DiffSteerState* state,
    double* vx,
    double* vy,
    double* vtheta)
{
    auto M = Matrix2();
    M(0,0) = 0.5;               M(0,1) = 0.5;
    M(1,0) = 0.5 / model->W;    M(1,1) = -0.5 / model->W;
    auto v = Vector2(M * model->r * Vector2(state->sr[1], state->sr[0]));
    *vx = v(0);
    *vy = 0.0;
    *vtheta = v(1);
}

void ComputeWheelFK(
    const AckermanSteerModel* model,
    const AckermanSteerState* state,
    double vx,
    double vy,
    double vtheta,
    Vector2 vcw[4],
    double tsr[4],
    double tst[4])
{
    if ((vx == 0.0) & (vtheta == 0.0)) {
        vcw[0] = vcw[1] = vcw[2] = vcw[3] = Vector2::Zero();
        tsr[0] = tsr[1] = tsr[2] = tsr[3] = 0.0;

        // NOTE: any set of wheel angles will work, but we'll return the current
        // wheel angles as a convenience
        tst[0] = state->st[0];
        tst[1] = state->st[1];
        tst[2] = state->st[2];
        tst[3] = state->st[3];

        return;
    }

    if (vtheta == 0.0) {
        tst[0] = tst[1] = tst[2] = tst[3] = 0.0;
        tsr[0] = tsr[1] = tsr[2] = tsr[3] = vx / model->r;

        vcw[0] = vx * Vector2(std::cos(tst[0]), std::sin(tst[0]));
        vcw[1] = vx * Vector2(std::cos(tst[1]), std::sin(tst[1]));
        vcw[2] = vx * Vector2(std::cos(tst[2]), std::sin(tst[2]));
        vcw[3] = vx * Vector2(std::cos(tst[3]), std::sin(tst[3]));

        return;
    }

    auto M = Matrix2();
    M(0,0) = 1.0; M(0,1) = 0.0;
    M(1,0) = 0.0; M(1,1) = model->L;

    auto alpha = atan2(vtheta * model->L, vx);

//    auto R = vx / vtheta; //model->L * vx / model->W;
    auto R = model->L / tan(alpha);

    // radius of curvature for each wheel
    double Rs[4];
    if (R >= 0.0) {
        Rs[0] = Vector2(model->L, R - 0.5 * model->W).norm();
        Rs[1] = Vector2(model->L, R + 0.5 * model->W).norm();
        Rs[2] = R - 0.5 * model->W;
        Rs[3] = R + 0.5 * model->W;
    } else {
        Rs[0] = -Vector2(model->L, 0.5 * model->W - R).norm();
        Rs[1] = -Vector2(model->L, -0.5 * model->W - R).norm();
        Rs[2] = -(0.5 * model->W - R);
        Rs[3] = -(-0.5 * model->W - R);
    }

    // speed of each wheel
    double Vs[4];
    Vs[0] = Rs[0] * vtheta;
    Vs[1] = Rs[1] * vtheta;
    Vs[2] = Rs[2] * vtheta;
    Vs[3] = Rs[3] * vtheta;

    if (R >= 0.0) {
        tst[0] = atan2(model->L, R - 0.5 * model->W);
        tst[1] = atan2(model->L, R + 0.5 * model->W);
    } else {
        tst[0] = atan2(-model->L, (0.5 * model->W - R));
        tst[1] = atan2(-model->L, (-0.5 * model->W - R));
    }
    tst[2] = 0.0;
    tst[3] = 0.0;

    tsr[0] = Vs[0] / model->r;
    tsr[1] = Vs[1] / model->r;
    tsr[2] = Vs[2] / model->r;
    tsr[3] = Vs[3] / model->r;

    // NOTE: we don't actually need these, just fill them out for rendering
    vcw[0] = Vs[0] * Vector2(std::cos(tst[0]), std::sin(tst[0]));
    vcw[1] = Vs[1] * Vector2(std::cos(tst[1]), std::sin(tst[1]));
    vcw[2] = Vs[2] * Vector2(std::cos(tst[2]), std::sin(tst[2]));
    vcw[3] = Vs[3] * Vector2(std::cos(tst[3]), std::sin(tst[3]));
}

void ComputeWheelIK(
    const AckermanSteerModel* model,
    const AckermanSteerState* state,
    double* vx,
    double* vy,
    double* vtheta)
{
    Matrix<double, 8, 3> H;
    H(0,0) = 1; H(0,1) = 0; H(0,2) = -( 0.5 * model->W);
    H(1,0) = 0; H(1,1) = 1; H(1,2) =  (       model->L);
    H(2,0) = 1; H(2,1) = 0; H(2,2) = -(-0.5 * model->W);
    H(3,0) = 0; H(3,1) = 1; H(3,2) =  (       model->L);
    H(4,0) = 1; H(4,1) = 0; H(4,2) = -( 0.5 * model->W);
    H(5,0) = 0; H(5,1) = 1; H(5,2) =  (            0.0);
    H(6,0) = 1; H(6,1) = 0; H(6,2) = -(-0.5 * model->W);
    H(7,0) = 0; H(7,1) = 1; H(7,2) =  (            0.0);

    Matrix<double, 8, 1> v;
    v(0) = model->r * state->sr[0] * std::cos(state->st[0]);
    v(1) = model->r * state->sr[0] * std::sin(state->st[0]);
    v(2) = model->r * state->sr[1] * std::cos(state->st[1]);
    v(3) = model->r * state->sr[1] * std::sin(state->st[1]);
    v(4) = model->r * state->sr[2] * std::cos(state->st[2]);
    v(5) = model->r * state->sr[2] * std::sin(state->st[2]);
    v(6) = model->r * state->sr[3] * std::cos(state->st[3]);
    v(7) = model->r * state->sr[3] * std::sin(state->st[3]);

    // (3x8 * 8x3)^-1 * 3x8 * 8x1 = 3x1
    auto V = Matrix<double, 3, 1>((H.transpose() * H).inverse() * H.transpose() * v);
    *vx = V(0);
    *vy = V(1);
    *vtheta = V(2);
}

} // namespace smpl
