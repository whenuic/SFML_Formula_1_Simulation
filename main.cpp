#include <SFML/Graphics.hpp>
#include <iostream>
#include <memory>
#include <thread>


#include "Spline.h"
#include "CubicSpline.h"
#include "WenSpline.h"
#include "Car.h"
#include "DriverPanel.h"
#include "RaceManager.h"
#include "RacePanel.h"
#include "ThreadPool.h"





int main() {
  // Create the main window
  float window_width = 1280.f;
  float window_height = 900.f;
  float track_view_width = 1024.f;
  float track_view_height = 768.f;

  int number_of_car_update_threads = 8;
  ThreadPool thread_pool(number_of_car_update_threads);

  sf::RenderWindow app(sf::VideoMode(window_width, window_height), "SFML window");
  // Set window position on the screen.
  app.setPosition(sf::Vector2i(550, 20));

  tgui::Gui gui(app);

  sf::View view(sf::Vector2f(-340.f, -450.f), sf::Vector2f(track_view_width * 1.35, track_view_height * 1.35));
  view.setViewport(
      sf::FloatRect(0.f, 0.f, track_view_width / window_width, track_view_height / window_height));
  app.setView(view);

  sf::Font font;
  if (!font.loadFromFile("DroidSansMono.ttf")) {
    std::cout << "Font load failed: "
              << "DroidSansMono.ttf" << std::endl;
  }

  sf::Texture circuit_texture;
  circuit_texture.loadFromFile("A1_Ring.png");
  // circuit_texture.loadFromFile("Shanghai_International_Circuit.png");
  sf::Vector2u circuit_texture_size = circuit_texture.getSize();
  LOG("Pic size " << circuit_texture_size.x << "x" << circuit_texture_size.y);
  sf::Sprite circuit_sprite;
  circuit_sprite.setTexture(circuit_texture);
  circuit_sprite.setOrigin(circuit_texture_size.x/2, circuit_texture_size.y/2);
  circuit_sprite.setPosition(-326, -460);
  circuit_sprite.setScale(1.4, 1.4);

  // Create FPS label
  sf::Text fps_text;
  fps_text.setFont(font);
  fps_text.setCharacterSize(20);
  fps_text.setFillColor(sf::Color::Green);
  fps_text.setPosition(-340.f, -450.f);

  // Create track
  float track_interpolation_interval = 10; // in meters
  // Mesh block interval = track_interpolation_interval / mesh_ratio
  int mesh_ratio = 8;
  Track track(&app, &font);
  track.SetInterval(track_interpolation_interval);
  track.LoadTrack(TRACKNAME::A1_RING, mesh_ratio);

  // >1 means fast forward, <1 means slow down
  float simulation_rate = 1;

  int num_of_cars = 20;
  std::vector<std::shared_ptr<Car>> cars;
  CarSpecificParams car_param;
  CarDisplayParams car_display_params = {/*draw_car_label = */false,
                                         /*draw_car_sensor_beam = */false};

  // Ferrari Car parameter
  CarSpecificParams car_1_params;
  car_1_params.car_number = 5;
  car_1_params.max_drive_force = 15000;
  car_1_params.max_brake_force = 40000;
  car_1_params.wheelbase = 3.7;
  car_1_params.length = 5.5;
  car_1_params.width = 1.8;
  car_1_params.body_drag_coeff = 0.75;
  car_1_params.wing_down_coeff = 6.4;
  car_1_params.wing_drag_coeff = 0.75;
  car_1_params.team_name = "Ferrari";
  car_1_params.color = sf::Color::Red;

  // Mercedes car parameter
  CarSpecificParams car_2_params;
  car_2_params.car_number = 44;
  car_2_params.max_drive_force = 15450;
  car_2_params.max_brake_force = 40000;
  car_2_params.wheelbase = 3.65;
  car_2_params.length = 5.5;
  car_2_params.width = 1.8;
  car_2_params.body_drag_coeff = 0.75;
  car_2_params.wing_down_coeff = 6.4;
  car_2_params.wing_drag_coeff = 0.75;
  car_2_params.team_name = "Mercedes";
  car_2_params.color = sf::Color::Cyan;

  for (int i=0; i<num_of_cars; i++) {
    car_param.max_drive_force = 14500 + rand() % 1001;
    car_param.max_brake_force = 39500 + rand() % 1001;
    car_param.wheelbase = 3.65 + float(rand() % 21) / 100;
    car_param.width = 1.8;
    car_param.length = 5.5;
    car_param.body_drag_coeff = 0.73 + float(rand() % 41) / 1000;
    car_param.wing_down_coeff = 6.3 + (rand() % 21) / 100;
    car_param.wing_drag_coeff = 0.73 + float(rand() % 21) / 100;
    car_param.team_name = "Dummy";
    car_param.color = sf::Color::White;
    if (i < 19 ) car_param = car_1_params;
    if (i == 19) {
      car_param = car_2_params;
      car_param.color = sf::Color::Green;
    }
    if (i == 18) {
      car_param.input_policy_name = INPUTPOLICYNAME::WAYPOINTS;
      car_param.color = sf::Color::White;
    } else if (i == 19) {
      car_param.input_policy_name = INPUTPOLICYNAME::DYNAMIC_TARGET_POINT;
    } else {
      car_param.input_policy_name = INPUTPOLICYNAME::DYNAMIC_TARGET_POINT;
    }
    car_param.car_number = i + 1;
    cars.push_back(std::make_unique<Car>(Car(&app, &font, car_param, car_display_params)));
    cars.back()->MountTyre(TYRETYPE::C1);
  }

  LOG("Car parameters set.");


  for (int i=0; i<cars.size(); i++) {
    track.RegisterCar(cars[i].get(), {TRACKOBJECTTYPE::GRID, static_cast<float>(i)});
  }

  std::vector<WayPoint> waypoints(36);
  waypoints[0].distance_on_track = 545;
  waypoints[0].bias = -6;
  waypoints[1].distance_on_track = 576;
  waypoints[1].bias = 6.5;
  waypoints[2].distance_on_track = 900;
  waypoints[2].bias = -5;
  waypoints[3].distance_on_track = 1240;
  waypoints[3].bias = 0;
  waypoints[4].distance_on_track = 1520;
  waypoints[4].bias = -6;
  waypoints[5].distance_on_track = 1540;
  waypoints[5].bias = 5;
  waypoints[6].distance_on_track = 1580;
  waypoints[6].bias = 0;
  waypoints[7].distance_on_track = 1690;
  waypoints[7].bias = 2;
  waypoints[8].distance_on_track = 1850;
  waypoints[8].bias = 3;
  waypoints[9].distance_on_track = 2100;
  waypoints[9].bias = -1;
  waypoints[10].distance_on_track = 2365;
  waypoints[10].bias = 0;
  waypoints[11].distance_on_track = 2400;
  waypoints[11].bias = 4;
  waypoints[12].distance_on_track = 2450;
  waypoints[12].bias = 0;
  waypoints[13].distance_on_track = 2530;
  waypoints[13].bias = 0;
  waypoints[14].distance_on_track = 2600;
  waypoints[14].bias = 2;
  waypoints[15].distance_on_track = 2680;
  waypoints[15].bias = 2;
  waypoints[16].distance_on_track = 2850;
  waypoints[16].bias = 5;
  waypoints[17].distance_on_track = 2920;
  waypoints[17].bias = 0;
  waypoints[18].distance_on_track = 2970;
  waypoints[18].bias = 0;
  waypoints[19].distance_on_track = 3050;
  waypoints[19].bias = 5;
  waypoints[20].distance_on_track = 3175;
  waypoints[20].bias = 5;
  waypoints[21].distance_on_track = 3230;
  waypoints[21].bias = -3;
  waypoints[22].distance_on_track = 3270;
  waypoints[22].bias = 0;
  waypoints[23].distance_on_track = 3320;
  waypoints[23].bias = 0;
  waypoints[24].distance_on_track = 3400;
  waypoints[24].bias = 0;
  waypoints[25].distance_on_track = 3480;
  waypoints[25].bias = 5;
  waypoints[26].distance_on_track = 3800;
  waypoints[26].bias = -5;
  waypoints[27].distance_on_track = 3950;
  waypoints[27].bias = -5;
  waypoints[28].distance_on_track = 4000;
  waypoints[28].bias = 2;
  waypoints[29].distance_on_track = 4050;
  waypoints[29].bias = 0;
  waypoints[30].distance_on_track = 4100;
  waypoints[30].bias = -5;
  waypoints[31].distance_on_track = 4150;
  waypoints[31].bias = -5;
  waypoints[32].distance_on_track = 4200;
  waypoints[32].bias = -5;
  waypoints[33].distance_on_track = 4250;
  waypoints[33].bias = 0;
  waypoints[34].distance_on_track = 4300;
  waypoints[34].bias = -5;
  waypoints[35].distance_on_track = 4350;
  waypoints[35].bias = -5;
  cars[18]->SetWaypoints(waypoints);


  // Create driver panel after car is created.
  std::unique_ptr<DriverPanel> driver_panel_1 = std::make_unique<DriverPanel>(sf::Vector2f(0.f, track_view_height), track_view_width/2, window_height - track_view_height, &app, &font, &gui, 15);
  cars[19]->SetDriverPanel(driver_panel_1.get());

  sf::Clock clock;
  float selected_point_moving_speed_gain = 500;

  float max_delta_step_allowed = 0.1f;

  // Create race panel
  RacePanel race_panel(sf::Vector2f(track_view_width, 0.f),
                       window_width - track_view_width, window_height, &app,
                       &font, &gui, 15, cars.size());

  // Create Race Manager and register cars
  RaceManager race(&thread_pool);
  race.SetRacePanel(&race_panel);
  for (auto& c : cars) {
    race.RegisterCar(c);
  }
  race.InitializeRace(70);

  // Start the game loop
  while (app.isOpen()) {
    // Process events
    sf::Event event;
    while (app.pollEvent(event)) {
      // Close window : exit
      if (event.type == sf::Event::Closed)
        app.close();

      if (event.type == sf::Event::Resized) {
        std::cout << "Window resized." << std::endl;
      }

      if (event.type == sf::Event::LostFocus) {
        std::cout << "LOST FOCUS." << std::endl;
      }

      if (event.type == sf::Event::GainedFocus) {
        std::cout << "GAINED FOCUS." << std::endl;
      }

      gui.handleEvent(event);


      if (event.type == sf::Event::MouseButtonPressed) {
        if (event.mouseButton.button == sf::Mouse::Left) {
          LOG("{" << event.mouseButton.x << ", " << event.mouseButton.y << "}");
          // sp.AddPoint({event.mouseButton.x, -event.mouseButton.y});
          // sp.DisplayError();
        }

        if (event.mouseButton.button == sf::Mouse::Right) {
          // sp.SelectPoint({event.mouseButton.x, -event.mouseButton.y});
        }
      }

      if (event.type == sf::Event::KeyPressed) {
        if(event.key.code == sf::Keyboard::Z) {
          view.zoom(1.2);
          app.setView(view);
        } else if (event.key.code == sf::Keyboard::X) {
          view.zoom(1/1.2);
          app.setView(view);
        } else if (event.key.code == sf::Keyboard::Up) {
          view.move(0, -5);
          app.setView(view);
        } else if (event.key.code == sf::Keyboard::Down) {
          view.move(0, 5);
          app.setView(view);
        } else if (event.key.code == sf::Keyboard::Left) {
          view.move(-5, 0);
          app.setView(view);
        } else if (event.key.code == sf::Keyboard::Right) {
          view.move(5, 0);
          app.setView(view);
        } else if (event.key.code == sf::Keyboard::Delete) {
          // sp.RemoveSelectedPoint();
        } else if (event.key.code == sf::Keyboard::P) {
          race.TogglePause();
        } else if (event.key.code == sf::Keyboard::Period) {
          simulation_rate *= 2.f;
        } else if (event.key.code == sf::Keyboard::Comma) {
          simulation_rate /= 2.f;
        }
      }
    }

    if (race.IsPaused()) continue;
    // Clear screen
    app.clear(sf::Color{50, 50, 50, 50});

    auto dt = clock.restart().asSeconds();
    fps_text.setString("FPS:" + utils::ToStringWithPrecision(1/dt, 0));
    // LOG("Time between frame: " << dt);
    if (dt > max_delta_step_allowed) {
      LOG("dt is greater than max_delta_step_allowed.");
      continue;
    }
    app.draw(fps_text);

    // app.draw(circuit_sprite);
    for (auto& c : cars) {
      c->Draw();
    }

    track.DrawTrack();

    // Update the window
    gui.draw();
    app.display();

    race.Update(dt, simulation_rate);

    if (0) {
      // Is joystick #0 connected?
      bool connected = sf::Joystick::isConnected(0);
      if (connected) {
        std::cout << "Joystick connected." << std::endl;
      }
      // How many buttons does joystick #0 support?
      unsigned int buttons = sf::Joystick::getButtonCount(0);
      std::cout << buttons << " supported." << std::endl;
      // Does joystick #0 define a X axis?
      bool hasX = sf::Joystick::hasAxis(0, sf::Joystick::X);
      std::cout << "X " << hasX << std::endl;
      // Is button #2 pressed on joystick #0?
      for(unsigned int i=0; i<buttons; i++) {
        bool pressed = sf::Joystick::isButtonPressed(0, i);
        if (pressed) {
          std::cout << "Button " << i << " pressed." << std::endl;
        }
      }
    }

  }

  return EXIT_SUCCESS;
}
