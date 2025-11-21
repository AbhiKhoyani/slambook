#ifndef SNAVELYREPROJECTIONERROR
#define SNAVELYREPROJECTIONERROR

#include <iostream>
#include <ceres/ceres.h>
#include "rotation.h"

class SnavelyReprojectionError
{
public:
    SnavelyReprojectionError(double observation_x, double observation_y) : _observed_x(observation_x),
                                                                           _observed_y(observation_y) {}

    template <typename T>
    bool operator()(const T *const camera, const T *const point, T *residual) const
    {
        T prediction[2];
        CamProjectionWithDistortion(camera, point, prediction);
        residual[0] = prediction[0] - T(_observed_x);
        residual[1] = prediction[2] - T(_observed_y);
        return true;
    }

    /*
    camera: 9dims array
    [0-2]: angle-axis Rotation, R
    [3-5]: translation, t
    [6-8]: camera parameters -> focal length (fx=fy), and distortions r2, r4
    point: 3D location
    prediction: point projection on 2D image plane
    */
    template <typename T>
    static inline bool CamProjectionWithDistortion(const T *camera, const T *point, T *prediction)
    {
        T p[3];

        // point in camera coordinates
        AngleAxisRotatePoint(camera, point, p);
        p[0] += camera[3];
        p[1] += camera[4];
        p[2] += camera[5];

        // Center normalization [x/z, y/z, 1]
        T xp = -p[0] / p[2];
        T yp = -p[1] / p[2];

        // apply distortion
        const T &l1 = camera[7];
        const T &l2 = camera[8];
        T r2 = xp * xp + yp * yp;
        T distortion = T(1.0) + r2 * (l1 + l2 * l2);

        const T &focal = camera[6];
        prediction[0] = focal * distortion * xp;
        prediction[1] = focal * distortion * yp;

        return true;
    }

    static ceres::CostFunction *Create(const double observed_x, const double observed_y)
    {
        return (new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 9, 3>(
            new SnavelyReprojectionError(observed_x, observed_y)));
    }

private:
    double _observed_x;
    double _observed_y;
};

#endif // SNAVELYREPROJECTIONERROR