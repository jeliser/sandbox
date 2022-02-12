
#include "gtest/gtest.h"

#include <memory>
#include <ACM.hpp>

TEST(ACM, SampleValidation) {
  const auto NONE = "NO PLANET";

  auto model = std::make_unique<ACM>();

  // Test the sample input and output from the challenge prompt
  EXPECT_EQ(ACM::Coordinates_t({0, 0, 0}), model->get_coordinates());
  EXPECT_EQ(NONE, model->get_planet());
  model->step(3, 4, 5);
  EXPECT_EQ(ACM::Coordinates_t({3, 4, 5}), model->get_coordinates());
  EXPECT_EQ("GRACE", model->get_planet());
  model->step({-4, 0, -6});
  EXPECT_EQ(ACM::Coordinates_t({-1, 4, -1}), model->get_coordinates());
  EXPECT_EQ("MROW", model->get_planet());
  model->step({18, -5, 3});
  EXPECT_EQ(ACM::Coordinates_t({17, -1, 2}), model->get_coordinates());
  EXPECT_EQ("BRAY", model->get_planet());
}

TEST(ACM, ZeroValidation) {
  const auto NONE = "NO PLANET";

  auto model = std::make_unique<ACM>();

  // Test all the zero conditions
  EXPECT_EQ(ACM::Coordinates_t({0, 0, 0}), model->get_coordinates());
  EXPECT_EQ(NONE, model->get_planet());
  model->step(1, 0, 0);
  EXPECT_EQ(ACM::Coordinates_t({1, 0, 0}), model->get_coordinates());
  EXPECT_EQ(NONE, model->get_planet());
  model->step(-1, 1, 0);
  EXPECT_EQ(ACM::Coordinates_t({0, 1, 0}), model->get_coordinates());
  EXPECT_EQ(NONE, model->get_planet());
  model->step(1, 0, 0);
  EXPECT_EQ(ACM::Coordinates_t({1, 1, 0}), model->get_coordinates());
  EXPECT_EQ(NONE, model->get_planet());
  model->step(-1, -1, -1);
  EXPECT_EQ(ACM::Coordinates_t({0, 0, -1}), model->get_coordinates());
  EXPECT_EQ(NONE, model->get_planet());
  model->step(-1, 0, 2);
  EXPECT_EQ(ACM::Coordinates_t({-1, 0, 1}), model->get_coordinates());
  EXPECT_EQ(NONE, model->get_planet());
  model->step(1, 1, -2);
  EXPECT_EQ(ACM::Coordinates_t({0, 1, -1}), model->get_coordinates());
  EXPECT_EQ(NONE, model->get_planet());

}

TEST(ACM, PlanetValidation) {
  const auto NONE = "NO PLANET";

  auto model = std::make_unique<ACM>();

  // Test all the planet conditions
  EXPECT_EQ(ACM::Coordinates_t({0, 0, 0}), model->get_coordinates());
  EXPECT_EQ(NONE, model->get_planet());
  model->step(100, 200, 300);
  EXPECT_EQ(ACM::Coordinates_t({100, 200, 300}), model->get_coordinates());
  EXPECT_EQ("GRACE", model->get_planet());
  model->step(-99, -300, 0);
  EXPECT_EQ(ACM::Coordinates_t({1, -100, 300}), model->get_coordinates());
  EXPECT_EQ("BRAY", model->get_planet());
  model->step(15, 101, -301);
  EXPECT_EQ(ACM::Coordinates_t({16, 1, -1}), model->get_coordinates());
  EXPECT_EQ("PRICE", model->get_planet());
  model->step(-10, -2, -1999);
  EXPECT_EQ(ACM::Coordinates_t({6, -1, -2000}), model->get_coordinates());
  EXPECT_EQ("MIG", model->get_planet());

  model->step(-7, 0, 0);
  EXPECT_EQ(ACM::Coordinates_t({-1, -1, -2000}), model->get_coordinates());
  EXPECT_EQ("SEBAS", model->get_planet());
  model->step(-49, 250, 1500);
  EXPECT_EQ(ACM::Coordinates_t({-50, 249, -500}), model->get_coordinates());
  EXPECT_EQ("MROW", model->get_planet());
  model->step(-50, -300, 1001);
  EXPECT_EQ(ACM::Coordinates_t({-100, -51, 501}), model->get_coordinates());
  EXPECT_EQ("TURK", model->get_planet());
  model->step(-134, 371, 563);
  EXPECT_EQ(ACM::Coordinates_t({-234, 320, 1064}), model->get_coordinates());
  EXPECT_EQ("WIEM", model->get_planet());

  model->step(234, -320, -1064);
  EXPECT_EQ(ACM::Coordinates_t({0, 0, 0}), model->get_coordinates());
  EXPECT_EQ(NONE, model->get_planet());
}
