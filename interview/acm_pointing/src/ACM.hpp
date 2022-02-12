#ifndef ACM_HPP
#define ACM_HPP

// system includes
#include <iostream>
#include <map>
#include <cmath>

// The Attitude Control Motor class
class ACM {

public:

  // Coordinates (internal state)
  typedef struct Coordinates_t {
    // Requirement was a 3 axis ACM
    int32_t x;
    int32_t y;
    int32_t z;

    // Allows user to add coordinates using the operater+=
    void operator+=(const Coordinates_t& rhs) {
      x += rhs.x;
      y += rhs.y;
      z += rhs.z;
    }

    // Allows a coordinate equality check using the operater==
    bool operator==(const Coordinates_t& rhs) const {
      return x == rhs.x && y == rhs.y && z == rhs.z;
    }

    // Alows pretty printing of the data structure.
    friend std::ostream& operator<<(std::ostream& os, const Coordinates_t& c) {
      return os << c.x << ", " << c.y << ", " << c.z;
    }
  } Coordinates_t;

public:

  ACM() {
  }

  ~ACM() {
  }

  // Give the user the option of sending the parameters in individually
  void step(const int32_t x, const int32_t y, const int32_t z) {
    step({x, y, z});
  }

  // Overload the step() method and allow for the coordinate structure as a parameter
  void step(const Coordinates_t& coordinates) {
    _coordinates += coordinates;
  }

  // Returns the internal state of the coordinates
  Coordinates_t get_coordinates() const {
    return _coordinates;
  }

  // Returns the planet we are current pointing to.
  std::string get_planet() const {
    // Make sure we have a planet to return
    if(_coordinates.x == 0 || _coordinates.y == 0 || _coordinates.z == 0) {
      return "NO PLANET";
    }

    // Choose which lookup to perform and normalize the Y and Z values.
    const auto pair = std::make_pair(_coordinates.y/abs(_coordinates.y), _coordinates.z/abs(_coordinates.z));
    return _coordinates.x > 0 ? _planets_p.find(pair)->second : _planets_n.find(pair)->second;
  }

private:

  // Internal instance of the coordinates
  Coordinates_t _coordinates = {0, 0, 0};

  // The planet lookups
  // Improvement: Load this lookup table dynamically from a YAML file
  std::map<std::pair<int32_t, int32_t>, std::string> _planets_p = {
    {{ 1,  1}, "GRACE"},
    {{-1,  1}, "BRAY"},
    {{ 1, -1}, "PRICE"},
    {{-1, -1}, "MIG"}
  };

  std::map<std::pair<int32_t, int32_t>, std::string> _planets_n = {
    {{ 1,  1}, "WIEM"},
    {{-1,  1}, "TURK"},
    {{ 1, -1}, "MROW"},
    {{-1, -1}, "SEBAS"}
  };

};


#endif /* ACM_HPP */
