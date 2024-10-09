#ifndef BANNER_H
#define BANNER_H

#include <TGUI/TGUI.hpp>
#include <memory>

#include "Car.h"
#include "Utils.h"

class Car;

class Banner {
 public:
  Banner(sf::Vector2f top_left_pos, float width, float height, int num_of_rows,
         int character_size, sf::RenderWindow* app, sf::Font* font,
         Car* car, tgui::Panel::Ptr panel);
  ~Banner();

  int GetNumOfRows();

  void SetText(std::string text, int line_number);

  bool Contains(sf::Vector2f position);
  sf::Vector2f GetPosition();
  void SetPosition(sf::Vector2f position);
  void RequestPushBack();

 private:
  sf::RenderWindow* app_;
  sf::Font* font_;

  sf::Vector2f top_left_position_;
  float width_;
  float height_;
  int num_of_rows_;
  int character_size_;

  std::vector<sf::Text> text_;
  sf::RectangleShape background_;
  sf::Color default_outline_color_ = sf::Color::White;
  sf::Color default_text_color_ = sf::Color::White;

  Car* car_;

  // tgui code
  tgui::Panel::Ptr parent_panel_;
  tgui::Panel::Ptr panel_;
  tgui::Button::Ptr pushback_;
  std::vector<tgui::Label::Ptr> labels_;
  tgui::ComboBox::Ptr gate_selector_;

};

#endif  // BANNER_H
