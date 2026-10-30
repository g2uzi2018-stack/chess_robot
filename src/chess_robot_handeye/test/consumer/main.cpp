#include <chess_robot_handeye/core.hpp>
int main() {
  const auto identity = chess_robot_handeye::T::eye();
  return chess_robot_handeye::translationDistance(
             identity, chess_robot_handeye::inverse(identity)) == 0.0 ? 0 : 1;
}
