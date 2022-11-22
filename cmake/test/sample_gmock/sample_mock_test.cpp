#include "gmock/gmock.h"

#define nullptr NULL

using ::testing::_;
using ::testing::AtLeast;
using ::testing::Return;

typedef struct {
  enum type { fail = 0u, success = 1u };
} return_val_t;

class Turtle {
  virtual return_val_t::type Walk() = 0;
  virtual uint32_t Eat(uint32_t amount) = 0;
};

#if 0
// URL gtest-master
class MockTurtle : public Turtle {
 public:
  MOCK_METHOD(void, Walk, (), (override));
  MOCK_METHOD(void, Eat, (), (override));
};
#endif

// URL gtest-1.8.0
class MockTurtleInterface : public Turtle {
 public:
  MOCK_METHOD0(Walk, return_val_t::type());
  MOCK_METHOD1(Eat, uint32_t(uint32_t));
};

class Jumper {
  public:
    MOCK_CONST_METHOD0(Jump, return_val_t::type*());
};

extern Jumper* jumper;

const return_val_t::type* Jump() {
  printf("HELLO %s\n", (jumper == nullptr) ? "TRUE" : "FALSE");
  return (jumper == nullptr) ? nullptr : jumper->Jump();
}

Jumper* jumper = nullptr;

/*
 * Test a simple interface class type
 */
TEST(MockTest, SampleTest_PASS) {
  // Instantiate the mock class
  MockTurtleInterface turtle;

  // Load all the expectations for how the class will be called
  EXPECT_CALL(turtle, Walk())
      .Times(AtLeast(3u))
      .WillOnce(Return(return_val_t::success))
      .WillRepeatedly(Return(return_val_t::fail));
  EXPECT_CALL(turtle, Eat(100u)).Times(AtLeast(1u)).WillOnce(Return(90u));

  // Call the class
  EXPECT_EQ(turtle.Walk(), return_val_t::success);
  EXPECT_EQ(turtle.Walk(), return_val_t::fail);
  EXPECT_EQ(turtle.Walk(), return_val_t::fail);
  EXPECT_EQ(turtle.Eat(100u), 90u);

  // Try the global extern
  Jumper grasshopper;
  jumper = &grasshopper;
  return_val_t::type ret_fail = return_val_t::fail;
  return_val_t::type ret_success = return_val_t::success;

  //EXPECT_EQ(grasshopper.Jump(), static_cast<return_val_t::type*>(nullptr));

  // This must be defined before the first to grasshopper.Jump()
  EXPECT_CALL(grasshopper, Jump())
      .Times(AtLeast(1u))
      .WillOnce(Return(&ret_fail))
      .WillRepeatedly(Return(&ret_success));

  EXPECT_EQ(*(grasshopper.Jump()), ret_fail);
  EXPECT_EQ(*(grasshopper.Jump()), ret_success);
  EXPECT_EQ(*(grasshopper.Jump()), ret_success);
}

/*
 * Test a simple interface class type
 */
TEST(MockTest, SampleTest_FAIL) {
  // Instantiate the mock class
  MockTurtleInterface turtle;

  // Load all the expectations for how the class will be called
  EXPECT_CALL(turtle, Walk()).Times(AtLeast(1));
  EXPECT_CALL(turtle, Eat(_)).Times(AtLeast(1));

  // Don't call the class ... we want to fail
}
