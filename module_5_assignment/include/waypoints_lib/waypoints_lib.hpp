#include <iostream>
#include <vector>
#include <cmath>
#include <memory>
#include <yaml-cpp/yaml.h>

using namespace std;

class WayPointLib
{
public:
    using Point = vector<float>;
    using Path = vector<vector<float>>;
    using XYTheta = vector<float>;
    using Coord2D = vector<float>;
    using Distance = float;

    WayPointLib() {};
    ~WayPointLib() {};
    Path make_path(XYTheta start, XYTheta goal, Path midpoints);
    Point getXY(XYTheta point);
    Distance compute_eucledian_distance(Coord2D x2y2, Coord2D x1y1);
    Distance compute_eucledian_distance(Path Path);
    u_int16_t path_no_selector(vector<Distance>);
    Path get_lower_energy_path(vector<Path> pathv);
    
};