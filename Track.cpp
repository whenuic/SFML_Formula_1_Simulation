#include "Track.h"
using namespace wenspline;


// Meshblock store file name.
const std::string kMeshBlockFileName = "MeshBlockFile.txt";

Track::Track(sf::RenderWindow* app, sf::Font* font)
  : app_(app),
    font_(font) {
  //ctor
}

Track::~Track() {
  //dtor
}

void Track::SetInterval(float dt) {
  dt_ = dt;
}

void Track::DrawTrack() {
  app_->draw(inner_line_, track_points_.size() + 1, sf::LineStrip);
  app_->draw(outer_line_, track_points_.size() + 1, sf::LineStrip);
  app_->draw(inner_line_pit_, pit_points_.size(), sf::LineStrip);
  app_->draw(outer_line_pit_, pit_points_.size(), sf::LineStrip);
  for (auto& tp : track_points_) {
    // tp.c and tp.c_fv are set in Helper2. If we don't run Helper2, then we
    // shouldn't draw these points.

    // app_->draw(tp.c);
    // app_->draw(tp.c_fv);
  }
  // app_->draw(triangle_strip_, 2 * track_points_.size() + 2, sf::LineStrip);
  for (auto& grid : grid_positions_) {
    for (auto& obj : grid.drawable_obj) {
      app_->draw(obj);
    }
  }
  /*for (auto& row : mesh_blocks_) {
    for (auto& mb : row) {
      app_->draw(mb.r);
    }
  }*/
}

Point Track::GetGridPosition(int num) {
  assert(num>=0 && num<=grid_positions_.size()-1);
  return grid_positions_[num].p;
}

float Track::GetGridGradient(int num) {
  assert(num>=0 && num<=grid_positions_.size()-1);
  return grid_positions_[num].gradient_angle;
}

void Track::RegisterCar(Car* car, TrackInitialPosition init_p) {
  assert(car != nullptr);
  assert(!track_points_.empty());
  car->SetTrack(this);
  switch (init_p.type) {
    case TRACKOBJECTTYPE::GRID:
      {
        Point grid_p = GetGridPosition(init_p.param);
        Point grid_pole = GetGridPosition(0.f);
        car->SetPosition(grid_p.x, grid_p.y);
        car->SetDynamicTargetPoint(grid_p.x, grid_p.y);
        car->SetOnTrack(true);
        car->SetLastKnownOnTrackPosition(grid_p.x, grid_p.y);
        car->SetHeading(GetGridGradient(init_p.param));
        float dist_on_track = GetDistanceOnTrack({grid_p.x, grid_p.y});
        float pole_dist_on_track =
            GetDistanceOnTrack({grid_pole.x, grid_pole.y});
        if (dist_on_track > pole_dist_on_track) {
          car->SetLapCompleted(-1);
        } else {
          car->ResetLapCompleted();
        }
        car->SetDistanceOnTrack(dist_on_track);
      }
      break;
    default:
      car->SetPosition(track_points_[0].p.x, track_points_[0].p.y);
      car->SetHeading(track_points_[0].gradient_angle);
      break;
  }
}

// Construct mesh blocks for On Track query
void Track::ConstructMeshBlocks(int thread_id,
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
                                const std::vector<sf::Vertex>& track_strip) {
  for (int i=i0; i<i1; i++) {
    for (int j=0; j<ny; j++) {
      // MeshBlock mb;
      mesh_blocks[i][j].x = meshblock_min_x + i*meshblock_dt;
      mesh_blocks[i][j].y = meshblock_min_y + j*meshblock_dt;
      // mesh_blocks[i][j].r.setSize(sf::Vector2f(meshblock_dt, meshblock_dt));
      // mesh_blocks[i][j].r.setOrigin(0, meshblock_dt);
      // mesh_blocks[i][j].r.setPosition(sf::Vector2f(mesh_blocks[i][j].x, -mesh_blocks[i][j].y));
      // for each corner, find in which triangle(s)
      std::vector<Point> corners;
      corners.push_back({mesh_blocks[i][j].x, mesh_blocks[i][j].y}); // bl
      corners.push_back({mesh_blocks[i][j].x + meshblock_dt, mesh_blocks[i][j].y}); // br
      corners.push_back({mesh_blocks[i][j].x + meshblock_dt, mesh_blocks[i][j].y + meshblock_dt}); // tr
      corners.push_back({mesh_blocks[i][j].x, mesh_blocks[i][j].y + meshblock_dt}); // tl
      std::unordered_map<int, int> triangle_index_2_occurrence;
      int total_on_track_corners = 0;
      for (int c=0; c<corners.size(); c++) {
        mesh_blocks[i][j].triangle_strip_index[c] = InWhichTrackTriangle(corners[c], track_strip);
        if (!mesh_blocks[i][j].triangle_strip_index[c].empty()) {
          total_on_track_corners++;
        }
        for (auto& id : mesh_blocks[i][j].triangle_strip_index[c]) {
          triangle_index_2_occurrence[id]++;
        }
        assert(mesh_blocks[i][j].triangle_strip_index[c].size() <= 3);
      }
      if (total_on_track_corners == 0) {
        mesh_blocks[i][j].on_track_indicator = -1;
        // mesh_blocks[i][j].r.setFillColor({0, 0, 0, 128}); // Black
        total_off_track_mesh_blocks[thread_id]++;
      } else if (total_on_track_corners > 0 && total_on_track_corners < 4) {
        mesh_blocks[i][j].on_track_indicator = 0;
        // mesh_blocks[i][j].r.setFillColor({255, 255, 0, 128}); // Yellow color
        total_uncertain_track_mesh_blocks[thread_id]++;
      } else if (total_on_track_corners == 4) {
        mesh_blocks[i][j].on_track_indicator = 1;
        // mesh_blocks[i][j].r.setFillColor({0, 255, 0, 128}); // Green color
        total_on_track_mesh_blocks[thread_id]++;
      }
    }
  }
}

void Track::LoadTrack(TRACKNAME track_name,
                      int mesh_ratio) {
  // Clear all containers
  track_points_.clear();
  pit_points_.clear();
  track_spline_info_.clear();
  pit_spline_info_.clear();
  track_strip_.clear();
  pit_strip_.clear();
  mesh_blocks_.clear();
  grid_positions_.clear();
  track_points_min_x_ = FLT_MAX;
  track_points_min_y_ = FLT_MAX;
  track_points_max_x_ = -FLT_MAX;
  track_points_max_y_ = -FLT_MAX;
  current_track_ = track_name;
  sp_.reset();
  sp_pit_.reset();
  sp_ = std::make_unique<wenspline::WenSpline>();
  sp_pit_ = std::make_unique<wenspline::WenSpline>();
  meshblock_dt_ratio_ = mesh_ratio;

  switch (track_name) {
  case TRACKNAME::A1_RING:
    float m = 1.41479605377;  // / 1.03234366095;
    width_ = 13.0f;
    width_pit_ = 5.0f;
    assert(dt_ > 0);
    sp_->SetInterval(dt_);
    sp_pit_->SetInterval(dt_);

    std::vector<wenspline::WenSplineParam> track_params;
    track_params.push_back(wenspline::WenSplineParam(WenSplineType::LINE, {-PI, {0, 0}}, WenSplineConnectingMode::USING_OWN_INIT_CONDITION, WenSplineBendDirection::DEFAULT, {388 * m}));
    track_params.push_back({ WenSplineType::ARC, {}, WenSplineConnectingMode::CONTINUOUS, WenSplineBendDirection::RIGHT, {20.f * m, PI / 180 * 76} });
    track_params.push_back({ WenSplineType::ARC, {}, WenSplineConnectingMode::CONTINUOUS, WenSplineBendDirection::LEFT, {350.f * m, PI / 180 * 9} });
    track_params.push_back({ WenSplineType::ARC, {}, WenSplineConnectingMode::CONTINUOUS, WenSplineBendDirection::RIGHT, {1200.f * m, PI / 180 * 16} });
    track_params.push_back({ WenSplineType::ARC, {}, WenSplineConnectingMode::CONTINUOUS, WenSplineBendDirection::LEFT, {320.f * m, PI / 180 * 21.3} });
    track_params.push_back({ WenSplineType::LINE, {}, WenSplineConnectingMode::CONTINUOUS, WenSplineBendDirection::DEFAULT, {152.f * m} });
    track_params.push_back({ WenSplineType::ARC, {}, WenSplineConnectingMode::CONTINUOUS, WenSplineBendDirection::RIGHT, {11.f * m, PI / 180 * 124} });
    track_params.push_back({ WenSplineType::ARC, {}, WenSplineConnectingMode::CONTINUOUS, WenSplineBendDirection::RIGHT, {615.f * m, PI / 180 * 21.8} });
    track_params.push_back({ WenSplineType::LINE, {}, WenSplineConnectingMode::CONTINUOUS, WenSplineBendDirection::DEFAULT, {50.f * m} });
    track_params.push_back({ WenSplineType::ARC, {}, WenSplineConnectingMode::CONTINUOUS, WenSplineBendDirection::LEFT, {1200.f * m, PI / 180 * 13} });
    track_params.push_back({ WenSplineType::ARC, {}, WenSplineConnectingMode::CONTINUOUS, WenSplineBendDirection::RIGHT, {25.f * m, PI / 180 * 110} });
    track_params.push_back({ WenSplineType::ARC, {}, WenSplineConnectingMode::CONTINUOUS, WenSplineBendDirection::RIGHT, {105.f * m, PI / 180 * 35} });
    track_params.push_back({ WenSplineType::ARC, {}, WenSplineConnectingMode::CONTINUOUS, WenSplineBendDirection::RIGHT, {180.f * m, PI / 180 * 45} });
    track_params.push_back({ WenSplineType::LINE, {}, WenSplineConnectingMode::CONTINUOUS, WenSplineBendDirection::DEFAULT, {100.f * m} });
    track_params.push_back({ WenSplineType::ARC, {}, WenSplineConnectingMode::CONTINUOUS, WenSplineBendDirection::LEFT, {50.f * m, PI / 180 * 120} });
    track_params.push_back({ WenSplineType::ARC, {}, WenSplineConnectingMode::CONTINUOUS, WenSplineBendDirection::LEFT, {470.f * m, PI / 180 * 17} });
    track_params.push_back({ WenSplineType::ARC, {}, WenSplineConnectingMode::CONTINUOUS, WenSplineBendDirection::LEFT, {40.f * m, PI / 180 * 83} });
    track_params.push_back({ WenSplineType::ARC, {}, WenSplineConnectingMode::CONTINUOUS, WenSplineBendDirection::LEFT, {125.f * m, PI / 180 * 22} });
    track_params.push_back({ WenSplineType::ARC, {}, WenSplineConnectingMode::CONTINUOUS, WenSplineBendDirection::RIGHT, {130.f * m, PI / 180 * 51.5} });
    track_params.push_back({ WenSplineType::LINE, {}, WenSplineConnectingMode::CONTINUOUS, WenSplineBendDirection::DEFAULT, {323.f * m} });
    track_params.push_back({ WenSplineType::ARC, {}, WenSplineConnectingMode::CONTINUOUS, WenSplineBendDirection::RIGHT, {float(47.405) * m, PI / 180 * 68} });
    track_params.push_back({ WenSplineType::ARC, {}, WenSplineConnectingMode::CONTINUOUS, WenSplineBendDirection::RIGHT, {485 * m, PI / 180 * 13} });
    track_params.push_back({ WenSplineType::ARC, {}, WenSplineConnectingMode::CONTINUOUS, WenSplineBendDirection::RIGHT, {42 * m, PI / 180 * 61} });
    track_params.push_back({ WenSplineType::ARC, {}, WenSplineConnectingMode::CONTINUOUS, WenSplineBendDirection::RIGHT, {100 * m, PI / 180 * 24} });
    track_params.push_back({ WenSplineType::LINE, {}, WenSplineConnectingMode::CONTINUOUS, WenSplineBendDirection::DEFAULT, {float(98.719) * m} });

    std::vector<wenspline::WenSplineParam> pit_params = {
        {WenSplineType::ARC, {-PI, {0, 0}}, WenSplineConnectingMode::USING_OWN_INIT_CONDITION, WenSplineBendDirection::RIGHT, {100 * m, PI / 180 * 30}},
        {WenSplineType::ARC, {}, WenSplineConnectingMode::CONTINUOUS, WenSplineBendDirection::LEFT, {100 * m, PI / 180 * 30}},
        {WenSplineType::LINE, {}, WenSplineConnectingMode::CONTINUOUS, WenSplineBendDirection::DEFAULT, {150 * m}},
        {WenSplineType::ARC, {}, WenSplineConnectingMode::CONTINUOUS, WenSplineBendDirection::LEFT, {100 * m, PI / 180 * 30}},
        {WenSplineType::ARC, {}, WenSplineConnectingMode::CONTINUOUS, WenSplineBendDirection::RIGHT, {100 * m, PI / 180 * 30}},
    };

    sp_->AddInstructions(track_params);
    sp_->Interpolate();
    sp_->GetInterpolatedPoints(track_points_);
    sp_->GetSplineInfo(track_spline_info_);
    sp_pit_->AddInstructions(pit_params);
    sp_pit_->Interpolate();
    sp_pit_->GetInterpolatedPoints(pit_points_);
    sp_pit_->GetSplineInfo(pit_spline_info_);

    for (auto& info : track_spline_info_) {
      LOG(std::to_string(info.accumulative_length) + " " +
          std::to_string(info.number_of_interpolated_points) + " " +
          std::to_string(info.length));
    }

    for (auto& info : pit_spline_info_) {
      LOG(std::to_string(info.accumulative_length) + " " +
          std::to_string(info.number_of_interpolated_points) + " " +
          std::to_string(info.length));
    }

    LOG(std::to_string(track_points_.size()) +
        " track points has been generated.");
    LOG(std::to_string(pit_points_.size()) +
        " pit points has been generated.");

    for (auto& pt : track_points_) {
      Point left, right;
      left.x = pt.p.x + width_ / 2 * cos(pt.gradient_angle + PI/2);
      left.y = pt.p.y + width_ / 2 * sin(pt.gradient_angle + PI/2);
      right.x = pt.p.x + width_ / 2 * cos(pt.gradient_angle - PI/2);
      right.y = pt.p.y + width_ / 2 * sin(pt.gradient_angle - PI/2);
      pt.p_left = {left.x, left.y};
      pt.p_right = {right.x, right.y};
      inner_line_[pt.index] = {sf::Vector2f(right.x, -right.y)};
      outer_line_[pt.index] = {sf::Vector2f(left.x, -left.y)};
      inner_line_[pt.index].texCoords = sf::Vector2f(pt.index, 1); // inner texCoords.y = 1
      outer_line_[pt.index].texCoords = sf::Vector2f(pt.index, -1); // outer texCoords.y = -1
      track_strip_.push_back(outer_line_[pt.index]);
      track_strip_.push_back(inner_line_[pt.index]);
      triangle_strip_[2 * pt.index] = outer_line_[pt.index];
      triangle_strip_[2 * pt.index].color = {255, 255, 255, 128};
      triangle_strip_[2 * pt.index + 1] = inner_line_[pt.index];
      triangle_strip_[2 * pt.index + 1].color = {255, 255, 255, 128};

      if (pt.p.x < track_points_min_x_) {
        track_points_min_x_ = pt.p.x;
      }
      if (pt.p.x > track_points_max_x_) {
        track_points_max_x_ = pt.p.x;
      }
      if (pt.p.y < track_points_min_y_) {
        track_points_min_y_ = pt.p.y;
      }
      if (pt.p.y > track_points_max_y_) {
        track_points_max_y_ = pt.p.y;
      }
    }
    // Adding the last two points to close the circuit loop.
    inner_line_[track_points_.size()] = inner_line_[0];
    outer_line_[track_points_.size()] = outer_line_[0];
    track_strip_.push_back(track_strip_[0]);
    track_strip_.push_back(track_strip_[1]);
    triangle_strip_[2 * track_points_.size()] = outer_line_[0];
    triangle_strip_[2 * track_points_.size() + 1] = inner_line_[0];
    triangle_strip_[2 * track_points_.size()].color = {255, 255, 255, 128};
    triangle_strip_[2 * track_points_.size() + 1].color = {255, 255, 255, 128};


    for (auto& pt : pit_points_) {
      Point left, right;
      left.x = pt.p.x + width_ / 2 * cos(pt.gradient_angle + PI/2);
      left.y = pt.p.y + width_ / 2 * sin(pt.gradient_angle + PI/2);
      right.x = pt.p.x + width_ / 2 * cos(pt.gradient_angle - PI/2);
      right.y = pt.p.y + width_ / 2 * sin(pt.gradient_angle - PI/2);
      pt.p_left = {left.x, left.y};
      pt.p_right = {right.x, right.y};
      inner_line_pit_[pt.index] = {sf::Vector2f(right.x, -right.y)};
      outer_line_pit_[pt.index] = {sf::Vector2f(left.x, -left.y)};
      inner_line_pit_[pt.index].texCoords = sf::Vector2f(pt.index, 1); // inner texCoords.y = 1
      outer_line_pit_[pt.index].texCoords = sf::Vector2f(pt.index, -1); // outer texCoords.y = -1
      pit_strip_.push_back(outer_line_pit_[pt.index]);
      pit_strip_.push_back(inner_line_pit_[pt.index]);
      triangle_strip_pit_[2 * pt.index] = outer_line_pit_[pt.index];
      triangle_strip_pit_[2 * pt.index].color = {255, 255, 255, 128};
      triangle_strip_pit_[2 * pt.index + 1] = inner_line_pit_[pt.index];
      triangle_strip_pit_[2 * pt.index + 1].color = {255, 255, 255, 128};

      if (pt.p.x < track_points_min_x_) {
        track_points_min_x_ = pt.p.x;
      }
      if (pt.p.x > track_points_max_x_) {
        track_points_max_x_ = pt.p.x;
      }
      if (pt.p.y < track_points_min_y_) {
        track_points_min_y_ = pt.p.y;
      }
      if (pt.p.y > track_points_max_y_) {
        track_points_max_y_ = pt.p.y;
      }
    }
    // Adding the last two points to close the circuit loop.
    //inner_line_[track_points_.size()] = inner_line_[0];
    //outer_line_[track_points_.size()] = outer_line_[0];
    //track_strip_.push_back(track_strip_[0]);
    //track_strip_.push_back(track_strip_[1]);
    //triangle_strip_[2 * track_points_.size()] = outer_line_[0];
    //triangle_strip_[2 * track_points_.size() + 1] = inner_line_[0];
    //triangle_strip_[2 * track_points_.size()].color = {255, 255, 255, 128};
    //triangle_strip_[2 * track_points_.size() + 1].color = {255, 255, 255, 128};

    // Determine the range of the mesh blocks
    sf::Clock clock_mesh_blocks; // mesh_ratio = 3, thread=16, use 14s; mesh_ratio=4,thread=16, use 23s, mesh_ratio=5,thread=16, use 37s
    bool compute_track = true; // compute in run time or load from file
    int total_on_track = 0;
    int total_off_track = 0;
    int total_uncertain = 0;

    if (compute_track) {
      float meshblock_min_x = track_points_min_x_ - 2 * std::max(dt_, width_);
      float meshblock_min_y = track_points_min_y_ - 2 * std::max(dt_, width_);
      float meshblock_dt = dt_ / meshblock_dt_ratio_;
      int nx = std::ceil((track_points_max_x_ - track_points_min_x_ +
                          4 * std::max(dt_, width_)) /
                         meshblock_dt);
      int ny = std::ceil((track_points_max_y_ - track_points_min_y_ +
                          4 * std::max(dt_, width_)) /
                         meshblock_dt);
      mesh_blocks_ = std::vector<std::vector<MeshBlock>>(
          nx, std::vector<MeshBlock>(ny, MeshBlock()));

      int number_of_threads = 16;
      std::vector<int> total_on_track_mesh_blocks(number_of_threads, 0);
      std::vector<int> total_off_track_mesh_blocks(number_of_threads, 0);
      std::vector<int> total_uncertain_track_mesh_blocks(number_of_threads, 0);

      std::vector<std::thread> threads;
      for (int i = 0; i < number_of_threads; i++) {
        threads.push_back(std::thread(
            ConstructMeshBlocks, i, i * nx / number_of_threads,
            (i + 1) * nx / number_of_threads, ny, std::ref(mesh_blocks_),
            meshblock_min_x, meshblock_min_y, meshblock_dt,
            std::ref(total_off_track_mesh_blocks),
            std::ref(total_on_track_mesh_blocks),
            std::ref(total_uncertain_track_mesh_blocks),
            std::ref(track_strip_)));
      }
      for (int i = 0; i < threads.size(); i++) {
        threads[i].join();
      }

      // Write computation result to file. If run helper 2, then need to move this function later.
      WriteMeshBlockToFile();

      LOG("Time to complete mesh block calculation: "
          << clock_mesh_blocks.restart().asSeconds());
      total_on_track = std::accumulate(total_on_track_mesh_blocks.begin(),
                                       total_on_track_mesh_blocks.end(), 0);
      total_off_track = std::accumulate(total_off_track_mesh_blocks.begin(),
                                        total_off_track_mesh_blocks.end(), 0);
      total_uncertain =
          std::accumulate(total_uncertain_track_mesh_blocks.begin(),
                          total_uncertain_track_mesh_blocks.end(), 0);
      LOG("Total on track mesh blocks: " << total_on_track);
      LOG("Total off track mesh blocks: " << total_off_track);
      LOG("Total uncertain track mesh blocks: " << total_uncertain);
      LOG("Num of meshblocks computed by threads: "
          << total_on_track + total_off_track + total_uncertain);
      LOG("Num of meshblocks in theory: " << nx * ny);
      assert(total_on_track + total_off_track + total_uncertain == nx * ny);
    } else {

      std::ifstream MeshBlockFile(kMeshBlockFileName);
      int d1, d2;
      int num_of_corners = 4;
      MeshBlockFile >> d1 >> d2;
      mesh_blocks_ = std::vector<std::vector<MeshBlock>>(
          d1, std::vector<MeshBlock>(d2, MeshBlock()));
      for (int i = 0; i < d1; i++) {
        int m, n;
        for (int j = 0; j < d2; j++) {
          MeshBlockFile >> m >> n;
          MeshBlockFile >> mesh_blocks_[m][n].x >> mesh_blocks_[m][n].y;
          int c;
          int strip_index;
          int on_track_corners = 0;
          for (int k = 0; k < num_of_corners; k++) {
            MeshBlockFile >> c;
            if (c == 0) {
              continue;
              // mesh_blocks_[m][n].triangle_strip_index[k].clear();
            } else {
              on_track_corners++;
              for (int l = 0; l < c; l++) {
                MeshBlockFile >> strip_index;
                mesh_blocks_[m][n].triangle_strip_index[k].push_back(strip_index);
              }
            }
          }
          if (on_track_corners == 0) {
            total_off_track++;
          } else if (on_track_corners == 4) {
            total_on_track++;
          } else if (on_track_corners < 4){
            total_uncertain++;
          } else {
            std::cerr << "on_track_corners = " << on_track_corners << " is wrong.";
          }
          MeshBlockFile >> mesh_blocks_[m][n].on_track_indicator;
        }
      }

      LOG("Time to load mesh block from file: "
          << clock_mesh_blocks.restart().asSeconds());
      LOG("Total on track mesh blocks: " << total_on_track);
      LOG("Total off track mesh blocks: " << total_off_track);
      LOG("Total uncertain track mesh blocks: " << total_uncertain);
      LOG("Num of meshblocks loaded from file: "
          << total_on_track + total_off_track + total_uncertain);
      LOG("Num of meshblocks in theory: " << d1 * d2);
      assert(total_on_track + total_off_track + total_uncertain == d1 * d2);
    }

    sf::Clock helper2_time;
    /*threads.clear();
    int num_of_track_points = track_points_.size();
    for (int i=0; i<number_of_threads; i++) {
      threads.push_back(std::thread(Helper2,
                                    i,
                                    i*num_of_track_points/number_of_threads,
                                    (i+1)*num_of_track_points/number_of_threads,
                                    std::ref(track_points_),
                                    std::ref(mesh_blocks_),
                                    std::ref(track_strip_),
                                    std::ref(dt_),
                                    std::ref(meshblock_dt_ratio_)));
    }
    for (int i=0; i<threads.size(); i++) {
      threads[i].join();
    }
    LOG("helper 2 time: " << helper2_time.restart().asSeconds());
    */

    track_length_ = track_spline_info_.back().accumulative_length + track_spline_info_.back().length;
    LOG("Track length: " + std::to_string(track_length_));

    // Set grid positions
    float grid_interval = 5; // 5 meters between pos 1 and pos 2
    float pole_distance = 80; // Pole grid in track distance
    float side_way_bias = 4; // left and right grid are 4*2 = 8 meters away
    for (int i=0; i<20; i++) {
      TrackObject grid;
      grid.type = TRACKOBJECTTYPE::GRID;
      sf::RectangleShape rs;
      rs.setFillColor(sf::Color::White);
      rs.setSize(sf::Vector2f(2.5, 0.25));
      rs.setOrigin(2.5/2, 0.25/2);
      Point p = {track_points_[0].p.x + (pole_distance-i*grid_interval) * cos(track_points_[0].gradient_angle),
                 track_points_[0].p.y + (pole_distance-i*grid_interval) * sin(track_points_[0].gradient_angle)};
      float gradient_angle = GetTrackDirection(p);
      if (utils::IsEven(i)) {
        // Even should be on the right
        p.x = p.x + side_way_bias * cos(gradient_angle + PI / 2);
        p.y = p.y + side_way_bias * sin(gradient_angle + PI / 2);
      } else {
        // Odd should be on the left
        p.x = p.x + side_way_bias * cos(gradient_angle - PI / 2);
        p.y = p.y + side_way_bias * sin(gradient_angle - PI / 2);
      }
      rs.setRotation(utils::ToDegree(gradient_angle - PI/2));
      rs.setPosition(p.x, -p.y);

      grid.p = p;
      grid.param = pole_distance - i*grid_interval;
      grid.drawable_obj.push_back(rs);
      grid.gradient_angle = gradient_angle;

      grid_positions_.push_back(grid);
    }

    break;
  }
}

float Track::GetTrackLength() {
  return track_length_;
}

// Find furthest view point on each circuit rib.
void Track::Helper2(int thread_id,
                    int i0,
                    int i1,
                    std::vector<WenSplineInterpolatedPoint>& track_points,
                    const std::vector<std::vector<MeshBlock>>& mesh_blocks,
                    const std::map<std::pair<int, int>, MeshBlock>& mesh_blocks_map,
                    const std::vector<sf::Vertex>& track_strip,
                    const float& dt,
                    const int& meshblock_dt_ratio) {
  for (int i=i0; i<i1; i++) {
    // set track points display circle
    track_points[i].c.setPosition(track_points[i].p.x, -track_points[i].p.y);
    if (track_points[i].spline_piece_index % 2 == 0) {
      track_points[i].c.setFillColor(sf::Color::Yellow);
    } else {
      track_points[i].c.setFillColor(sf::Color::Red);
    }
    // Compute furthest view point on the rib.
    int num_of_pts = 21;
    int num_of_view_angles = 1801; // 0.1 degrees increment
    int furthest_index = 0;
    float furthest_view_distance = 0;
    float furthest_view_angle = 0;
    Point furthest_view_point;
    Point left_to_right = {track_points[i].p_right.x - track_points[i].p_left.x, track_points[i].p_right.y - track_points[i].p_left.y};
    for (int i=0; i<num_of_pts; i++) {
      Point test_p = {track_points[i].p_left.x + left_to_right.x / (num_of_pts-1) * i, track_points[i].p_right.y + left_to_right.y / (num_of_pts-1) * i};
      for (int j=0; j<num_of_view_angles; j++) {
        float view_angle = track_points[i].gradient_angle - PI/2 + j*PI/(num_of_view_angles-1);
        float d = DistanceToBoundary(test_p, view_angle, 0.2, 1000, mesh_blocks, track_strip, dt, meshblock_dt_ratio);
        if (d > furthest_view_distance) {
          furthest_view_distance = d;
          furthest_view_angle = view_angle;
          furthest_view_point = test_p;
        }
      }
    }
    track_points[i].furthest_view_point = furthest_view_point;
    track_points[i].furthest_view_distance = furthest_view_distance;
    track_points[i].furthest_view_angle = furthest_view_angle;
    track_points[i].c_fv.setPosition(furthest_view_point.x, -furthest_view_point.y);
  }
}

bool IsPointInsideTriangle(Point p, Point a, Point b, Point c) {
		Point AB, AC, AP;
		AB = {b.x - a.x, b.y - a.y};
		AC = {c.x - a.x, c.y - a.y};
		AP = {p.x - a.x, p.y - a.y};
		float dot00 = AC.x * AC.x + AC.y * AC.y;
		float dot01 = AC.x * AB.x + AC.y * AB.y;
		float dot02 = AC.x * AP.x + AC.y * AP.y;
		float dot11 = AB.x * AB.x + AB.y * AB.y;
		float dot12 = AB.x * AP.x + AB.y * AP.y;
		float inverDeno = (dot00 * dot11 - dot01 * dot01);

		float u = (dot11 * dot02 - dot01 * dot12) / inverDeno;
		float v = (dot00 * dot12 - dot01 * dot02) / inverDeno;

		return (u >= 0) && (v >= 0) && (u + v <= 1);
}

std::vector<int> Track::InWhichTrackTriangle(Point p, const std::vector<sf::Vertex>& track_strip) {
  Point p0,p1,p2;
  std::vector<int> res;
  for (int i=0; i<track_strip.size()-2; i++) {
    p0.x = track_strip[i].position.x;
    p0.y = -track_strip[i].position.y;
    p1.x = track_strip[i+1].position.x;
    p1.y = -track_strip[i+1].position.y;
    p2.x = track_strip[i+2].position.x;
    p2.y = -track_strip[i+2].position.y;
    if (IsPointInsideTriangle(p, p0, p1, p2)) {
      res.push_back(i);
    }
  }
  return res;
}

bool Track::IsOnTrack(Point p, int& triangle_strip_index) {
  return IsOnTrack(p, triangle_strip_index, mesh_blocks_,
                   track_strip_, dt_, meshblock_dt_ratio_);
}

// This is a static function ready to be called by multi threading.
bool Track::IsOnTrack(Point p,
                      int& triangle_strip_index,
                      const std::vector<std::vector<MeshBlock>>& mesh_blocks,
                      const std::vector<sf::Vertex>& track_strip,
                      const float& dt,
                      const int& meshblock_dt_ratio) {
  triangle_strip_index = -1; // when this function returns false, set -1
  // LOG("IsOnTrack: " << p.x << " " << p.y);
  std::vector<int> x_index, y_index;
  float xd = (p.x - mesh_blocks[0][0].x) / (dt / meshblock_dt_ratio);
  float yd = (p.y - mesh_blocks[0][0].y) / (dt / meshblock_dt_ratio);
  if (xd < 0 || yd < 0 || xd > mesh_blocks.size() || yd > mesh_blocks[0].size()) {
    return false;
  }
  x_index.push_back(floor(xd));
  y_index.push_back(floor(yd));

  if (xd == floor(xd) && xd != 0) {
    x_index.push_back(xd-1);
  }
  if (yd == floor(yd) && yd != 0) {
    y_index.push_back(yd-1);
  }

  for (auto& xi : x_index) {
    for (auto& yi : y_index) {
      if (mesh_blocks[xi][yi].on_track_indicator == 1) {
        for (auto& r : mesh_blocks[xi][yi].triangle_strip_index) {
          for (auto& c : r) {
            if (c >= 0) {
              triangle_strip_index = c;
              return true;
            }
          }
        }
        assert(false);
      } else if (mesh_blocks[xi][yi].on_track_indicator == 0) {
        for (int i=0; i<mesh_blocks[xi][yi].triangle_strip_index.size(); i++) {
          for (int j=0; j<mesh_blocks[xi][yi].triangle_strip_index[i].size(); j++) {
            int strip_id = mesh_blocks[xi][yi].triangle_strip_index[i][j];
            if (strip_id != -1) {
              Point p1, p2, p3;
              p1.x = track_strip[strip_id].position.x;
              p1.y = -track_strip[strip_id].position.y;
              p2.x = track_strip[strip_id + 1].position.x;
              p2.y = -track_strip[strip_id + 1].position.y;
              p3.x = track_strip[strip_id + 2].position.x;
              p3.y = -track_strip[strip_id + 2].position.y;
              if (IsPointInsideTriangle(p, p1, p2, p3)) {
                triangle_strip_index = strip_id;
                return true;
              }
            }
          }
        }
      }
    }
  }
  return false;
}

float Track::DistanceToBoundary(Point p,
                                float direction_angle,
                                float precision,
                                float sensor_range) {
  return DistanceToBoundary(p, direction_angle, precision, sensor_range, mesh_blocks_, track_strip_, dt_, meshblock_dt_ratio_);
}

float Track::DistanceToBoundary(Point p,
                                float direction_angle,
                                float precision,
                                float sensor_range,
                                const std::vector<std::vector<MeshBlock>>& mesh_blocks,
                                const std::vector<sf::Vertex>& track_strip,
                                const float& dt,
                                const int& meshblock_dt_ratio) {
  // Maintain 2 distances, the boundary will be between these two points. Then
  // approach 2 points to the boundary until error goes below threshold.
  // Another method is to create a [0, 1, 2, 3, 4 ,5 ,6, ... , 30] vector and
  // check boundary is between which two points.

  Point p1;
  int inc = 1;
  int last_good_i = 0;
  int n = sensor_range / precision;
  int i = 0;
  while (i < n) {
    i+=inc;
    float range = i * precision;
    p1.x = p.x + range * cos(direction_angle);
    p1.y = p.y + range * sin(direction_angle);
    int triangle_strip_index = -1;
    if (IsOnTrack(p1, triangle_strip_index, mesh_blocks, track_strip, dt, meshblock_dt_ratio)) {
      last_good_i = i;
      inc = std::min(inc * 2, 50);
      if (i + inc > n-1) {
        inc = n-1-i;
        if (inc == 0) {
          break;
        }
      }
    } else {
      if (i == last_good_i + 1) {
        return last_good_i * precision;
      } else {
        i = last_good_i;
        inc = 1;
      }
    }
  }
  return sensor_range;
}

float Track::GetTrackDirection(Point p) {
  int triangle_strip_index = -1;
  IsOnTrack(p, triangle_strip_index);
  int tp_index = track_strip_[triangle_strip_index].texCoords.x;
  return track_points_[tp_index].gradient_angle;
}

Point Track::GetTrackCenterPoint(float distance) {
  while (distance > track_length_) {
    distance -= track_length_;
  }
  int index = std::floor(distance / dt_);
  float distance_remainder = distance - dt_ * index;
  Point cp = track_points_[index].p;
  float direction = track_points_[index].gradient_angle;
  return GetTrackCenterPoint({cp.x + std::cos(direction) * distance_remainder, cp.y + std::sin(direction) * distance_remainder});
}

Point Track::GetTrackCenterPoint(Point p) {
  int triangle_strip_index = -1;
  if (IsOnTrack(p, triangle_strip_index)) {
    // Find out the track_point index.
    int tp_index = track_strip_[triangle_strip_index].texCoords.x;
    return track_points_[tp_index].p;
  } else {
    return {FLT_MAX, FLT_MAX};
  }
}

// If not on track, return -1.f
float Track::GetDistanceOnTrack(Point p) {
  float distance_on_track = -1.f;
  int triangle_strip_index = -1;
  if (IsOnTrack(p, triangle_strip_index)) {
    int tp_index = track_strip_[triangle_strip_index].texCoords.x;
    bool is_inner = (track_strip_[triangle_strip_index].texCoords.y == 1);
    float gradient_angle = track_points_[tp_index].gradient_angle;
    Point to_p = {p.x - track_points_[tp_index].p.x, p.y - track_points_[tp_index].p.y};
    // If p is on a track_points, then early return the distance.
    distance_on_track = track_points_[tp_index].index * dt_;
    if (to_p.x != 0 || to_p.y == 0) {
      float angle_to_p = utils::PrincipalArgument(to_p);
      float phi;
      if (is_inner) {
        phi = utils::DiffAngleRadian(angle_to_p, gradient_angle - PI);
        distance_on_track -= sqrt(to_p.x * to_p.x + to_p.y * to_p.y) * cos(phi);
      } else {
        phi = utils::DiffAngleRadian(angle_to_p, gradient_angle);
        distance_on_track += sqrt(to_p.x * to_p.x + to_p.y * to_p.y) * cos(phi);
      }
      if (distance_on_track > track_length_) {
        LOG("triangle_strip_index: " + std::to_string(triangle_strip_index));
        LOG("tp_index: " + std::to_string(tp_index));
        LOG("before bias distance: " +
            std::to_string(track_points_[tp_index].index * dt_));
        LOG("p: " + std::to_string(p.x) + " " + std::to_string(p.y));
        LOG("angle_to_p: " + std::to_string(angle_to_p));
        LOG("phi: " + std::to_string(phi));
        LOG("is_inner: " + std::to_string(is_inner? 1 : 0));
        LOG("distance_on_track: " + std::to_string(distance_on_track));
      }
    }
  }
  return distance_on_track;
}

bool Track::IsCrossFinishLine(float x, float y, float dx, float dy) {
  // check p and p+dp on different side of finish line AND the both end points
  // of the extended finish line are on different side of p->p+dp
  Point p = {x,y};
  Point p1 = {x+dx, y+dy};
  Point zero = {track_points_[0].p.x, track_points_[0].p.y};

  if (p.x == zero.x && p.y == zero.y) {
    return false;
  }
  if (utils::Distance(p, zero) > 50 || utils::Distance(p1, zero) > 50) {
    return false;
  }

  float mid_angle = utils::ToNormalizedAngle(track_points_[0].gradient_angle - PI/2);
  float p_angle = utils::ToNormalizedAngle(utils::PrincipalArgument({p.x-zero.x, p.y-zero.y}));
  float p1_angle = utils::ToNormalizedAngle(utils::PrincipalArgument({p1.x-zero.x, p1.y-zero.y}));

  float diff_angle = utils::DiffAngleRadian(mid_angle, p_angle);
  float diff_angle1 = utils::DiffAngleRadian(p1_angle, mid_angle);


  if (diff_angle < PI && diff_angle1 < PI) {
    // LOG("x: " << x << " y: " << y << " dx: " << dx << " dy: " << dy);
    // LOG("mid: " << utils::ToDegree(mid_angle) << " p: " << utils::ToDegree(diff_angle) << " p1: " << utils::ToDegree(diff_angle1));
  }
  return (diff_angle < PI && diff_angle1 < PI);
}

void Track::WriteMeshBlockToFile() {
  std::ofstream MeshBlockFile(kMeshBlockFileName);
  MeshBlockFile << mesh_blocks_.size() << " " << mesh_blocks_[0].size()
                << std::endl;
  for (int i = 0; i < mesh_blocks_.size(); i++) {
    for (int j = 0; j < mesh_blocks_[i].size(); j++) {
      MeshBlockFile << i << " " << j << std::endl;
      MeshBlockFile << std::fixed << std::setprecision(25) << mesh_blocks_[i][j].x << " " << mesh_blocks_[i][j].y << std::endl;
      for (int m = 0; m < mesh_blocks_[i][j].triangle_strip_index.size(); m++) {
        int num_of_triangle = mesh_blocks_[i][j].triangle_strip_index[m].size();
        MeshBlockFile << num_of_triangle;
        for (int n = 0; n < num_of_triangle; n++) {
          MeshBlockFile << " " << mesh_blocks_[i][j].triangle_strip_index[m][n];
        }
        MeshBlockFile << std::endl;
      }
      MeshBlockFile << mesh_blocks_[i][j].on_track_indicator << std::endl;
    }
  }
}
