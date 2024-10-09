#ifndef PANEL_H
#define PANEL_H

#include <TGUI/TGUI.hpp>

class Panel {
 public:
  Panel(sf::Vector2f top_left_pos, float width, float height,
        sf::RenderWindow* app, sf::Font* font,
        tgui::Gui* gui, int text_size)
      : app_(app),
        font_(font),
        top_left_position_(top_left_pos),
        width_(width),
        height_(height),
        gui_(gui),
        text_size_(text_size) {
    // tgui code
    panel_ = tgui::Panel::create();
    panel_->getRenderer()->setBackgroundColor(sf::Color::Black);
    gui->add(panel_);
    panel_->setSize(width_, height_);
    panel_->setPosition({top_left_position_.x, top_left_position_.y});
  }

 protected:
  sf::RenderWindow* app_;
  sf::Font* font_;

  sf::Vector2f top_left_position_;
  float width_;
  float height_;
  int text_size_;

  sf::RectangleShape panel_frame_;
  sf::Color default_outline_color_ = sf::Color::White;
  sf::Color default_text_color_ = sf::Color::White;

  // -------------------------------------
  tgui::Gui* gui_;
  tgui::Panel::Ptr panel_;

};


#endif // PANEL_H