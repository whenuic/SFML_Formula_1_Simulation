#ifndef DRIVERPANEL_H
#define DRIVERPANEL_H

#include <TGUI/TGUI.hpp>
#include "Panel.h"

// Use labelname to find the label ptr in the vector.
enum LabelName {
  TEAM_NAME, // Team name
  DRIVER_NAME, // Name 3 letters + " #XX"(car number)
  SPEED,
  FUEL_MASS,
  BEST_LAP_TIME, // lap number + lap time + tyre type
  CURRENT_LAP_TIME, // current lap number + current lap time
  TYRE_CONDITION, // tyre condition + tyre type + laps used

  COUNT, // used to tell how many elements are used in the enum. This count number is used to initialized the size of the Labels container of the DriverPanel.
};

class DriverPanel : public Panel {
 public:
  DriverPanel(sf::Vector2f top_left_pos, float width, float height,
              sf::RenderWindow* app, sf::Font* font, tgui::Gui* gui, int text_size);

  bool Contains(sf::Vector2f position);
  void SetLabelText(LabelName name, std::string text);


 private:
  // x_pos and y_pos are percentage with respect of the panel width and height, width and height are also percentage with respect to the panel size.
  void SetLabelParams(LabelName name, float x_pos, float y_pos, float width, float height, sf::Color back_ground_color, sf::Color text_color);

 private:
  // The container of the labels, use label name to find the label. Label name is enum which is the index of the container.
  std::vector<tgui::Label::Ptr> labels_;
};

#endif  // DRIVERPANEL_H