#ifndef RACEPANEL_H
#define RACEPANEL_H

#include <TGUI/TGUI.hpp>
#include "Panel.h"
#include "RaceManager.h"

class RaceManager;

struct CarOrderInfo {
  int order;  // current position in the race, [1, number of cars]
  std::string name_display;  // three Upper case letters
  std::string interval;      // seconds behind the car in the position minus 1
  std::string type_label;    // one Upper case letter indicating which type
                             // compound is using
};

class RacePanel : public Panel {
 public:
  RacePanel(sf::Vector2f top_left_pos, float width, float height,
            sf::RenderWindow* app, sf::Font* font, tgui::Gui* gui,
            int text_size, int num_of_cars);

  void SetRaceManager(RaceManager* race_manager);
  void SetRankingText(std::vector<std::shared_ptr<Car>>& cars);
  void SetNumOfCars(int n);

 private:
  tgui::Button::Ptr pause_button_;
  RaceManager* race_manager_;

  std::vector<tgui::Label::Ptr> ranking_labels_;
  int num_of_cars_ = 0;

};


#endif // RACEPANEL_H