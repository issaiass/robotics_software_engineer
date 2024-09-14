#include "waypoints_lib/waypoints_lib.hpp"

WayPointLib::Path WayPointLib::make_path(XYTheta start, XYTheta goal, Path midpoints)
{
    Path path;
    path.push_back(start);
    for (auto it = midpoints.begin(); it != midpoints.end(); ++it)
    {
        path.push_back(*it);
    }
    path.push_back(goal);
    return path;
}

WayPointLib::Point WayPointLib::getXY(XYTheta point)
{
    Point slice(point.begin() + 0, point.begin() + 2);
    return slice;
}

WayPointLib::Distance WayPointLib::compute_eucledian_distance(Coord2D x2y2, Coord2D x1y1)
{
    return sqrt(pow(x2y2[0] - x1y1[0], 2) + pow(x2y2[1] - x1y1[1], 2));
}

WayPointLib::Distance WayPointLib::compute_eucledian_distance(Path p)
{
    Distance total_distance;

    Coord2D x1y1 = this->getXY(p[0]);
    total_distance = 0;
    for (auto it = p.begin() + 1; it != p.end(); it++)
    {
        Point x2y2 = getXY(*it);
        total_distance += compute_eucledian_distance(x2y2, x1y1);
        x1y1 = x2y2;
    }
    return total_distance;
}

uint16_t WayPointLib::path_no_selector(vector<Distance> d)
{
    return std::min_element(d.begin(), d.end()) - d.begin();
}

WayPointLib::Path WayPointLib::get_lower_energy_path(vector<WayPointLib::Path> pathv)
{
    WayPointLib::Distance min_distance = std::numeric_limits<WayPointLib::Distance>::max();
    WayPointLib::Path best_path;
    vector<Distance> distances;

    for (WayPointLib::Path p : pathv)
    {
        WayPointLib::Distance d = this->compute_eucledian_distance(p);
        distances.push_back(d);
        if (d < min_distance)
        {
            min_distance = d;
            best_path = p;
        }
    }
    cout << "Path " << path_no_selector(distances)+1 << " selected." << endl;
    return best_path;
}