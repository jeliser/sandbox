
#include "gtest/gtest.h"

#include <memory>
#include <ACS.hpp>

TEST(ACS, SampleValidation) {
  const auto NONE = "NO PLANET";

  auto model = std::make_unique<ACS>();

  EXPECT_EQ(ACS::Coordinates_t({0, 0, 0}), model->get_coordinates());
  EXPECT_EQ(NONE, model->get_planet());

  model->step(3, 4, 5);
  EXPECT_EQ(ACS::Coordinates_t({3, 4, 5}), model->get_coordinates());
  EXPECT_EQ("GRACE", model->get_planet());

  model->step({-4, 0, -6});
  EXPECT_EQ(ACS::Coordinates_t({-1, 4, -1}), model->get_coordinates());
  EXPECT_EQ("MROW", model->get_planet());

  model->step({18, -5, 3});
  EXPECT_EQ(ACS::Coordinates_t({17, -1, 2}), model->get_coordinates());
  EXPECT_EQ("BRAY", model->get_planet());
}

TEST(ACS, EdgeValidation) {


}
