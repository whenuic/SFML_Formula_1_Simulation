#include "RaceManager.h"
#include <chrono>

// Refresh rank every 1 second.
float kRankRefreshInterval = 1.f;

RaceManager::RaceManager(ThreadPool* thread_pool) :
  thread_pool_(thread_pool) {
}

void RaceManager::InitializeRace(int lap_total) {
  race_time_ = 0;
  is_paused_ = false;
  lap_total_ = lap_total;
  lap_completed_ = 0;
  fastest_lap_ = {nullptr, 0, "", 0};

  // Initialize ranking labels
  RankCars();
  race_panel_->SetNumOfCars(cars_.size());
  race_panel_->SetRankingText(cars_);

  return;
}

void RaceManager::Update(float dt, float simulation_rate) {
  if (is_paused_) {
    return;
  }

  float actual_dt = dt * simulation_rate;

  thread_pool_->SetAvailableTasksCount(cars_.size());
  for (int i = cars_.size() - 1; i >= 0; i--) {
    thread_pool_->EnqueueTask([=]() {
      cars_[i]->Update(actual_dt);
    });
  }

  // Need to wait for all the updates finished. Otherwise the car has very long updates might get less frequently updated.
  while (thread_pool_->GetAvailableTasksCount() > 0) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  rank_refresh_timer_ += actual_dt;
  if (rank_refresh_timer_ >= kRankRefreshInterval) {
    RankCars();
    rank_refresh_timer_ -= kRankRefreshInterval;
    while (rank_refresh_timer_ > 1) {
      rank_refresh_timer_ -= 1;
    }
    race_panel_->SetRankingText(cars_);
  }
  
}

// Bubble sort on the lab number then distance to start
void RaceManager::RankCars() {
  // 0, 1, ... size-1
  size_t size = cars_.size();
  std::shared_ptr<Car> tmp = nullptr;
  for (size_t i = 0; i < size - 1; i++) {
    for (size_t j = 0; j < size - 1 - i; j++) {
      if (cars_[j]->IsBehind(cars_[j+1].get())) {
        tmp = cars_[j];
        cars_[j] = cars_[j + 1];
        cars_[j + 1] = tmp;
      }
    }
  }
}

void RaceManager::StartRace() { is_paused_ = false; }

void RaceManager::StopRace() {

}

bool RaceManager::IsPaused() { return is_paused_; }

void RaceManager::TogglePause() { is_paused_ = !is_paused_; }

void RaceManager::RegisterCar(std::shared_ptr<Car> car) { 
  // No need to resetlapcompleted since Track::RegisterCar will set the lap_completed for each car.
  car->SetRaceManager(this);
  cars_.push_back(car);
}

void RaceManager::SetRacePanel(RacePanel* race_panel) {
  race_panel_ = race_panel;
  race_panel_->SetRaceManager(this);
}

int RaceManager::GetNumOfCars() { return cars_.size(); }