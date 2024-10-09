#include "DriverPanel.h"

DriverPanel::DriverPanel(sf::Vector2f top_left_pos, float width, float height,
                         sf::RenderWindow* app, sf::Font* font, tgui::Gui* gui, int text_size)
    : Panel(top_left_pos, width, height, app, font, gui, text_size) {

  for (int i = 0; i < LabelName::COUNT; i++) {
    tgui::Label::Ptr label = tgui::Label::create();
    panel_->add(label);
    labels_.push_back(label);
  }

  SetLabelParams(LabelName::SPEED, 0.f, 1.f / 6, 1.f / 4, 1.f / 6,
                 sf::Color::Green, sf::Color::White);
  SetLabelParams(LabelName::TEAM_NAME, 0.f, 0.f, 1.f / 4, 1.f / 6,
                 sf::Color::Red, sf::Color::White);
  SetLabelParams(LabelName::DRIVER_NAME, 1.f / 4, 0.f, 1.f / 4, 1.f / 6,
                 sf::Color::Red, sf::Color::White);
  SetLabelParams(LabelName::FUEL_MASS, 1.f / 4, 1.f / 6, 1.f / 4, 1.f / 6,
                 sf::Color::Green, sf::Color::White);
  SetLabelParams(LabelName::CURRENT_LAP_TIME, 3.f / 4, 0.f, 1.f / 4, 1.f / 6,
                 sf::Color::Red, sf::Color::White);
  SetLabelParams(LabelName::BEST_LAP_TIME, .5f, 0.f, 1.f / 4, 1.f / 6,
                 sf::Color::Magenta, sf::Color::White);
  SetLabelParams(LabelName::TYRE_CONDITION, 0.f, 3.f / 6, 1.f / 4, 1.f / 6,
                 sf::Color::Black, sf::Color::White);

}

bool DriverPanel::Contains(sf::Vector2f position) { return false; }

void DriverPanel::SetLabelParams(LabelName name, float x_pos, float y_pos,
                                 float width, float height,
                                 sf::Color back_ground_color,
                                 sf::Color text_color) {
  labels_[name]->setPosition({width_ * x_pos, height_ * y_pos});
  labels_[name]->setSize({width_ / width, height_ / height});
  labels_[name]->getRenderer()->setBackgroundColor(back_ground_color);
  labels_[name]->getRenderer()->setTextColor(text_color);
  labels_[name]->setTextSize(text_size_);
}

void DriverPanel::SetLabelText(LabelName label_name, std::string text) {
  labels_[label_name]->setText(text);
}