#ifndef TRACK_H
#define TRACK_H

#include <cfloat>
#include <unordered_map>
#include <thread>
#include <numeric>
#include <fstream>
#include <iomanip> // set fstream precision

#include "SFML/Graphics.hpp"
#include "Debug.h"
#include "WenSpline.h"
#include "Car.h"

// The track needs:
// 1. Assets - image, font, app
// 2. Display
// 3. API for driver to query
//    1) Is a point on the track[build mesh map to answer this query]
//       Mesh grid's length should be smaller than interval. Mesh grid contains
//       closest center point and track gradient.
//    2) Which center point is it closest to[according to mesh map]
//    3) How far is a point from the center line(- toward left, + toward right)[according to mesh map]
//    4) Distance to the start line[each interpolation point carries this info]
//    5) Cross start line check[depending on distance to the start line]
//    6) View query - distance to the wall in a specified direction
//    7) Close-by cars
// 4. Spline to interpolate the points
//    1) Center line points in equally spaced curve distance[may not depending on which spline to use]
//    2) First order derivative(direction of the track)[]
//    3) Second order derivative[]
//    4) Left and right boundary points[]
//    5) Track normal? point to the inner side of the track[]
//    6) radius[]
//    7) dist from start line[]
// 5. Track information
//    1) Start line
//    2) Sector line
//    3) DRS detection line
//    4) DRS enable line
//    5) Corner number
//    6) Corner start/end line
//    7) Pit track
//    8) Pit spot
//    9) Pit speed limit line
// 6. Inputs
//    1) app, font
//    2) picture
//    3) dt

enum TRACKNAME {
  NOT_LOADED = 0,
  A1_RING = 1,
};

// Name of different track objects
enum TRACKOBJECTTYPE {
  GRID = 0, // Start position from 1 - 20(even more)
  TRACK = 1, // Track point
  PIT_LANE = 2, // Pit drive way, different from pit stop lane
  PIT_STOP_LANE = 3, // All team pit stops are on the pit stop lane. Pit stop lane is next to the pit drive lane.
  PIT_STOP = 4, // Pit stop, one per team
  PIT_PARKING = 5, // Pit parking lot, one per car
  START_LINE = 6, // start/finish line
  SECTOR_LINE = 7, // sector lines
  DRS_DETECTION_POINT = 8, // check speed at this point to determine whether DRS will be enabled
  DRS_ENABLE_POINT = 9, // If allowed, DRS can be activated at this point.
  PIT_SPEED_LIMIT_LINE = 10, // Apply speed limit beyond this line during entering pit
  PIT_START_LINE = 11, // Pit exit, also serve as start from pit line
};

// Let the car to specify where is it to register the car to the track
struct TrackInitialPosition {
  TRACKOBJECTTYPE type = TRACKOBJECTTYPE::GRID;
  // For Grid, param is the grid position; For Track distance, it's the distance
  // from the start line; For PIT, it's the pit position.
  float param = 0;
};

//
struct TrackObject {
  TRACKOBJECTTYPE type = TRACKOBJECTTYPE::TRACK;
  float param = 0;
  Point p;
  float gradient_angle = 0;
  std::vector<sf::RectangleShape> drawable_obj;
};

// The meshblock is for fast detection of wether a point is on the track or not.
struct MeshBlock {
  // bottom left x and y, dx and dy are defined by the Track private member
  // variables.
  float x = 0;
  float y = 0;
  // The corners of this meshblock are inside which triangle strip of the track.
  // Corners go from bottom left, bottom right, top right, top left(in
  // counter-clock wise order). If all 4 corners are inside the same triangle,
  // then any point inside this mesh block must be in the same triangle. If the
  // corners are in different triangles, any point inside this mesh block should
  // be test against all triangles to find out if it is inside any of them. If
  // none of the corners are inside a triangle, then any point inside this mesh
  // block is not on the track. On the track, a point may be inside at most 3
  // triangles if it's an outside or inside boundary point.
  // [i][j] i-which corner(4 of mesh block), j-inside which triangle(3 at most)
  std::vector<std::vector<int>> triangle_strip_index =
                      std::vector<std::vector<int>>(4);
  // -1: not on track, 0: need to test against the triangles, +1: on track
  int on_track_indicator = -1;
  // sf::RectangleShape r;
};

class Car;

class Track
{
  public:
    Track(sf::RenderWindow* app, sf::Font* font);
    virtual ~Track();

    // Car interaction
    void RegisterCar(Car* car, TrackInitialPosition init_p);

    // Track management
    void LoadTrack(TRACKNAME track_name, int mesh_ratio);
    void DrawTrack();
    void SetInterval(float dt);

    // Track query
    bool IsCrossFinishLine(float x, float y, float dx, float dy);
    // Check which mesh block(4 at most) does this point belong to, if necessary
    // do a fine check. It also write which triangle is the point in. Two
    // versions because need to run multi-thread calculation.
    bool IsOnTrack(Point p,
                   int& triangle_strip_index);
    static bool IsOnTrack(Point p,
                   int& triangle_strip_index,
                   const std::vector<std::vector<MeshBlock>>& mesh_blocks,
                   const std::vector<sf::Vertex>& track_strip,
                   const float& dt,
                   const int& meshblock_dt_ratio);

    // Given point p and its direction angle in radian, return the distance to
    // the circuit boundary in such direction. The caller must make sure p is on
    // on the track. dt is the biggest error the caller can accept. Notice the
    // sensor has a max range at 100m.
    // Two versions for multi-thread call.
    float DistanceToBoundary(Point p,
                             float direction_angle,
                             float precision,
                             float sensor_range);
    static float DistanceToBoundary(Point p,
                                    float direction_angle,
                                    float precision,
                                    float sensor_range,
                                    const std::vector<std::vector<MeshBlock>>& mesh_blocks,
                                    const std::vector<sf::Vertex>& track_strip,
                                    const float& dt,
                                    const int& meshblock_dt_ratio);

    // Return the track distance associated with Point p
    float GetDistanceOnTrack(Point p);
    // Return the total length of the track
    float GetTrackLength();

    // Get the position's track direction
    float GetTrackDirection(Point p);
    // Get the track center point(track_points_[some index]) of a position. If input position is not on track, return {FLT_MAX, FLT_MAX}.
    Point GetTrackCenterPoint(Point p);
    // Get the track center point based on track distance
    Point GetTrackCenterPoint(float distance);

    // Get grid position, pole position = 0
    Point GetGridPosition(int num);
    // Get grid num's gradient angle
    float GetGridGradient(int num);

    // Write MeshBlock vector data to the file, hopefully it will be much more faster to load this data from file than from real time computation
    void WriteMeshBlockToFile();



  private:
    static std::vector<int>
             InWhichTrackTriangle(Point p,
                                  const std::vector<sf::Vertex>& track_strip);

    // Construct mesh blocks for On Track query
    static void ConstructMeshBlocks(int thread_id,
                                    int i0,
                                    int i1,
                                    int ny,
                                    std::vector<std::vector<MeshBlock>>& mesh_blocks,
                                    const float meshblock_min_x,
                                    const float meshblock_min_y,
                                    const float meshblock_dt,
                                    std::vector<int>& total_off_track_mesh_blocks,
                                    std::vector<int>& total_on_track_mesh_blocks,
                                    std::vector<int>& total_uncertain_track_mesh_blocks,
                                    const std::vector<sf::Vertex>& track_strip);

    // Find furthest view point on each circuit rib.
    static void Helper2(int thread_id,
                        int i0,
                        int i1,
                        std::vector<WenSplineInterpolatedPoint>& track_points,
                        const std::vector<std::vector<MeshBlock>>& mesh_blocks,
                        const std::map<std::pair<int, int>, MeshBlock>& mesh_blocks_map,
                        const std::vector<sf::Vertex>& track_strip,
                        const float& dt,
                        const int& meshblock_dt_ratio);

  private:
    sf::RenderWindow* app_;
    sf::Font* font_;

    // Curve distance(using curve integral not distance between points)
    // between 2 interpolation points on the track center line
    float dt_;
    float width_;
    float width_pit_;
    float track_length_ = 0;
    float pit_length_ = 0;

    // Spline object this track owns
    std::unique_ptr<wenspline::WenSpline> sp_;
    // Spline object of the pit
    std::unique_ptr<wenspline::WenSpline> sp_pit_;

    TRACKNAME current_track_;




    std::vector<WenSplineInterpolatedPoint> track_points_;
    std::vector<WenSplineInterpolatedPoint> pit_points_;
    // The range of track_points_. Large negative number should be -FLT_MAX.
    // FLT_MIN is smallest positive number above zero.
    float track_points_min_x_ = FLT_MAX;
    float track_points_min_y_ = FLT_MAX;
    float track_points_max_x_ = -FLT_MAX;
    float track_points_max_y_ = -FLT_MAX;
    std::vector<wenspline::WenSplineInfo> track_spline_info_;
    std::vector<wenspline::WenSplineInfo> pit_spline_info_;
    sf::Vertex inner_line_[5000];
    sf::Vertex outer_line_[5000];
    sf::Vertex triangle_strip_[10000];
    sf::Vertex inner_line_pit_[1000];
    sf::Vertex outer_line_pit_[1000];
    sf::Vertex triangle_strip_pit_[2000];
    // Store left and right boundary points of track_points_ in sequence.
    // The last two strip vertices are the copy of the first two.
    // This gives the drawing a closed curve.
    // Reuse the texCoords.x to store which track_points is this vertex
    // corresponding to. use texCoords.y to store inner(+1, right) or
    // outer(-1, left).
    std::vector<sf::Vertex> track_strip_;
    std::vector<sf::Vertex> pit_strip_;

    std::vector<std::vector<MeshBlock>> mesh_blocks_;
    // How many meshblocks fit in one dt_
    int meshblock_dt_ratio_ = 4;

    // Store grid positions and drawable objects. These are the positions car start in the race.
    std::vector<TrackObject> grid_positions_;

};

#endif // TRACK_H
