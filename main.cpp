#include <iostream>
#include "Eigen/Eigen"

// calculate the coordinate (x, y, z) of d in frame [a, b, c]
void project2F(const Eigen::Vector3d &a, const Eigen::Vector3d &b, const Eigen::Vector3d &c,
               const Eigen::Vector3d &d, double &x, double &y, double &z)
{
    z = d.dot(c);                      // vector c is a unit vector
    Eigen::Vector3d e = d - z*c;       // note e as the foot point of d on plane abc
    Eigen::Matrix2d coeff;
    coeff << a[0], b[0], a[1], b[1];
    Eigen::Vector2d e2(e[0], e[1]);    // note e2 as the first 2 dimension of e
    Eigen::Vector2d xy = coeff.inverse() * e2;
    x = xy[0];
    y = xy[1];
}


int main()
{
    Eigen::Vector3d A(0, 0, 0);
    Eigen::Vector3d B(1, 0, 0);
    Eigen::Vector3d C(0, 1, 0);
    Eigen::Vector3d E(1, 1, 1);
    Eigen::Vector3d F(0, 0, -1);
    std::pair<Eigen::Vector3d, Eigen::Vector3d> segmet(E, F);
    std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d> triangle(A, B, C);

    Eigen::Vector3d a = std::get<0>(triangle);
    Eigen::Vector3d b = std::get<1>(triangle);
    Eigen::Vector3d c = std::get<2>(triangle);
    Eigen::Vector3d e = segmet.first;
    Eigen::Vector3d f = segmet.second;

    bool isInter = true;

    // Step1: construct a coordinate frame whose unit vertor is ab, ac and ab x ac
    Eigen::Vector3d ab = b - a;
    Eigen::Vector3d ac = c - a;
    Eigen::Vector3d ad = ab.cross(ac);
    ad.normalize();      // ad is the unit vector that perpendicular to ab and ac, now ab, ac
                         // and ad make up a Cartesian coordinate frame F

    // Step2: compute the coordinate of point e and f in frame F
    Eigen::Vector3d ae = e - a;
    double x1, y1, z1;
    project2F(ab, ac, ad, ae, x1, y1, z1);
    Eigen::Vector3d af = f - a;
    double x2, y2, z2;
    project2F(ab, ac, ad, af, x2, y2, z2);

    // Step3: judge if ef intersects with plane abc
    // if z1 and z2 are both positive or negative, ef will not intersect with plan abc
    if (z1 * z2 > 0)
    {
        isInter = false;
    }

    // step4: if ef intersect with plane abc, judge if it intersect with triangle abc
    // step4.1: get the coordinate of intersection point p, as p is on plane abc, set z = 0, calculate x,y
    double x = x1 - (x2 - x1)/(z2 - z1) * z1;
    double y = y1 - (y2 - y1)/(z2 - z1) * z1;

    // step4.2: judge if p is in triangle abc
    // if (x, y) is not in first quadrant, it will not be in triangle abc
    if (x < 0 || y < 0)
    {
        isInter = false;
    }

    // if (x, y) is above line x + y = 1, it is also not in triangle abc
    if (x + y > 1)
    {
        isInter = false;
    }

    if (isInter)
    {
        std::cout << "intersect";
    }
    else
    {
        std::cout << "not intersect";
    }

    return 0;
}