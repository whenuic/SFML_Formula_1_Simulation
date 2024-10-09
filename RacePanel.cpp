#include "RacePanel.h"

RacePanel::RacePanel(sf::Vector2f top_left_pos, float width, float height,
                     sf::RenderWindow* app, sf::Font* font, tgui::Gui* gui,
                     int text_size, int num_of_cars)
    : Panel(top_left_pos, width, height, app, font, gui, text_size) {
  num_of_cars_ = num_of_cars;
  

  pause_button_ = tgui::Button::create();
  panel_->add(pause_button_);
  pause_button_->setSize({width_ / 8, height_ / 50});
  pause_button_->setPosition({0,0});
  pause_button_->setText("P/R");
  pause_button_->setTextSize(text_size_);
  pause_button_->getRenderer()->setBackgroundColor("Green");
  pause_button_->getRenderer()->setTextColor(default_text_color_);
  pause_button_->onPress([&]() {
    race_manager_->TogglePause();
    std::cout << "Pause button pressed.";
  });
  pause_button_->setEnabled(true);

  // Initialize ranking labels
  for (int i = 0; i < num_of_cars_; i++) {
    tgui::Label::Ptr label = tgui::Label::create();
    label->getRenderer()->setBackgroundColor(sf::Color::Green);
    label->getRenderer()->setTextColor(default_text_color_);
    label->setSize({width_, height_ / 50});
    label->setPosition({0, pause_button_->getSize().y + i * label->getSize().y});
    panel_->add(label);
    label->setText(std::to_string(i + 1));
    ranking_labels_.push_back(label);
  }
}

void RacePanel::SetRaceManager(RaceManager* race_manager) {
  race_manager_ = race_manager;
}

void RacePanel::SetRankingText(std::vector<std::shared_ptr<Car>>& cars) {
  for (int i = 0; i < num_of_cars_; i++) {
    ranking_labels_[i]->setText(std::to_string(i+1) + " " + std::to_string(cars[i]->GetCarNumber()) + " " + "DUM " + utils::ToLapTime(cars[i]->GetLastLapTime()));
  }

}

void RacePanel::SetNumOfCars(int n) { num_of_cars_ = n; }