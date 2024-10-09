#ifndef SPLINE_H
#define SPLINE_H

#include <SFML/Graphics.hpp>
#include "Car.h"
#include <unordered_map>
#include "Debug.h"
#include "CommonDataStructure.h"

class Car;


class Spline {
  friend class Car;

  public:
    Spline(sf::RenderWindow* app, sf::Font* font, float dt, float pixel_to_dist_gain);

    void AddPoint(Point p);
    void AddPoints(const std::vector<Point>& pts);
    void AddPointAt(Point p, int index);

    void RemovePoint(Point p);
    void RemoveSelectedPoint();

    void SelectPoint(Point p);
    void SelectPreviousPoint();
    void SelectNextPoint();

    void MoveSelectedPoint(float dx, float dy);

    void DisplayError();
    void ConstructRacingLine();

    void UpdateCar(float dt);
    void Draw();

    void RegisterCar(Car* car);

    bool IsCrossFinishLine(float old_x, float old_y, float dx, float dy);

    void SetPitPoints(const std::vector<Point> pts, int in_index, int out_index);

    virtual ~Spline();

  private:
    void UpdateSplines(std::vector<ControlPoint>& control_points,
                       std::vector<InterpolatedPoint>& interpolated_points,
                       std::vector<sf::Vertex>& triangle_strip,
                       std::map<std::pair<std::pair<float, float>,
                       std::pair<float, float>>,
                       int>& bound2index,
                       float width,
                       bool is_loop);
    void Interpolate(float t,
                     bool is_loop,
                     InterpolatedPoint& ip,
                     const std::vector<ControlPoint>& cps,
                     const std::vector<InterpolatedPoint>& ips,
                     float width);
    float GetGradientAngle(float t, bool is_loop);
    float Integral(const InterpolatedPoint& p, const InterpolatedPoint& q);

    void HighlightPoint(int index);
    void UnHighlightPoint(int index);

    float DistanceToT(float distance);
    float DistanceToT2(float distance);

    // True if and only if p is inside triangle or on the edge of the triangle
    bool IsPointInsideTriangle(Point p, Point p0, Point p1, Point p2); // p0 p1 p2 are triangle vertices
    int InWhichTriangle(Point p); // in which triangle strip

  private:
    sf::RenderWindow* app_;
    sf::Font* font_;

    // t is 0-1 between two control points, dt_ is the interval between two interpolated points, dt_ should be less than 0.5
    float dt_;

    float select_point_distance_threshold_ = 3.0f;

    int selected_control_point_index_ = 0;

    std::vector<ControlPoint> control_points_;
    std::vector<InterpolatedPoint> interpolated_points_;
    std::vector<sf::Vertex> track_strip_;
    // <outpoint, insidepoint> -> interpolated point index
    std::map<std::pair<std::pair<float, float>, std::pair<float, float>>, int> track_bound2index_;
    float track_width_;

    sf::CircleShape car_;
    float car_t_ = 0;
    float car_d_ = 0;
    float car_speed_;
    sf::Text car_speed_text_;
    sf::Text lap_time_text_;
    sf::Text dt_text_;
    float lap_time_;
    std::vector<float> laps_time_;

    std::vector<ControlPoint> racing_line_control_points_;
    std::vector<InterpolatedPoint> racing_line_interpolated_points_;

    // Both ends of pit lane coincide with track control points
    std::vector<ControlPoint> pit_control_points_;
    std::vector<InterpolatedPoint> pit_interpolated_points_;
    std::vector<sf::Vertex> pit_strip_;
    // <outpoint, insidepoint> -> interpolated point index
    std::map<std::pair<std::pair<float, float>, std::pair<float, float>>, int> pit_bound2index_;
    float pit_width_;

    float pixel_to_dist_gain_;
};

#endif // SPLINE_H
