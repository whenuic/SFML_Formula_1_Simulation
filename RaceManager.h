#ifndef RACEMANAGER_H
#define RACEMANAGER_H

#include <vector>
#include <condition_variable>
#include <mutex>
#include "Car.h"
#include "RacePanel.h"
#include "ThreadPool.h"

class RacePanel;

struct FastestLap {
  std::shared_ptr<Car> car; // which car created this fastest lap
  float lap_time; // fastest lap time in float
  std::string lap_time_string; // fastest lap time in mm:ss.mmm format
  int lap; // in which lap does this lap time created
};



class RaceManager {
 public:
  RaceManager(ThreadPool* thread_pool);
  
  void InitializeRace(int lap_total);

  void StartRace();
  void StopRace();

  void Update(float dt, float simulation_rate);

  // RankCars will be called every 1 second or maybe even less.
  void RankCars();

  void RegisterCar(std::shared_ptr<Car> car);
  int GetNumOfCars();

  bool IsPaused();
  void TogglePause();

  void SetRacePanel(RacePanel* race_panel);

 private:


 private:
  std::vector<std::shared_ptr<Car>> cars_;

  ThreadPool* thread_pool_; // Thread pool to call cars.Update() function, not owned.
  
  // Used with the condition variable to indicate all cars finished update function in the current racemanager.update function. If not check, since the update are assigned to the threads but not joined in the thread running racemanager, some car.update that takes longer to finish will be called less frequently.
  // (TODO)

  bool is_paused_ = false;

  int lap_total_ = 20;
  int lap_completed_ = 0;
  FastestLap fastest_lap_ = {nullptr, 0, "", 0};


  float race_time_ = 0.f; // time elapsed since the race starts
  float rank_refresh_timer_ = 0.f; // rank refresh every 1 second

  // -------------------------------------
  RacePanel* race_panel_ = nullptr; // not owned
};



#endif // RACEMANAGER_H