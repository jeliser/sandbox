#ifndef ACS_HPP
#define ACS_HPP

#include <iostream>
#include <map>
#include <cmath>

class ACS {

public:

  // Coordinates (internal state)
  typedef struct Coordinates_t {
    // Requirement was a 3 axis ACM
    int32_t x;
    int32_t y;
    int32_t z;

    void operator+=(const Coordinates_t& rhs) {
      x += rhs.x;
      y += rhs.y;
      z += rhs.z;
    }

    bool operator==(const Coordinates_t& rhs) const {
      return x == rhs.x && y == rhs.y && z == rhs.z;
    }

    friend std::ostream& operator<<(std::ostream& os, const Coordinates_t& c) {
      return os << c.x << ", " << c.y << ", " << c.z;
    }
  } Coordinates_t;

public:

  ACS() {
  }

  ~ACS() {
  }

  void step(const int32_t x, const int32_t y, const int32_t z) {
    step({x, y, z});
  }

  void step(const Coordinates_t& coordinates) {
    _coordinates += coordinates;
  }

  Coordinates_t get_coordinates() const {
    return _coordinates;
  }

  std::string get_planet() const {
    // Make sure we have a planet to return
    if(_coordinates.x == 0 || _coordinates.y == 0 || _coordinates.z == 0) {
      return "NO PLANET";
    }

    // Choose which lookup to perform
    const auto pair = std::make_pair(_coordinates.y/abs(_coordinates.y), _coordinates.z/abs(_coordinates.z));
    return _coordinates.x > 0 ? _planets_p.find(pair)->second : _planets_n.find(pair)->second;
  }

private:

  Coordinates_t _coordinates = {0, 0, 0};

  // Improvement: Load this lookup table dynamically from a YAML file
  // The planet lookups
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


#endif /* ACS_HPP */
