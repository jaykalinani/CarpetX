// Standalone check of the published RK4-2 and RK4-3 coefficients.
// Compile: g++ -std=c++17 -Wall -Werror -o /tmp/check_rk_coeffs check_rk_coeffs.cxx

#include "../src/RK4-2_coeffs.hpp"
#include "../src/RK4-3_coeffs.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {
using namespace MultiStepRungeKutta;

void expect_eq(const char *name, double got, double expect) {
  if (!(std::abs(got - expect) <= 1e-15 * std::max(1.0, std::abs(expect)))) {
    std::cerr << name << " got " << got << " expected " << expect << "\n";
    std::exit(1);
  }
}

struct rk42 {
  double c2, c3, b0, b1, b2, a20, a30, a31;
};

rk42 sol1() {
  return {rk4_dash_2_sol_1_c2<double>(), rk4_dash_2_sol_1_c3<double>(),
          rk4_dash_2_sol_1_b0<double>(), rk4_dash_2_sol_1_b1<double>(),
          rk4_dash_2_sol_1_b2<double>(), rk4_dash_2_sol_1_a20<double>(),
          rk4_dash_2_sol_1_a30<double>(), rk4_dash_2_sol_1_a31<double>()};
}

rk42 sol2() {
  return {rk4_dash_2_sol_2_c2<double>(), rk4_dash_2_sol_2_c3<double>(),
          rk4_dash_2_sol_2_b0<double>(), rk4_dash_2_sol_2_b1<double>(),
          rk4_dash_2_sol_2_b2<double>(), rk4_dash_2_sol_2_a20<double>(),
          rk4_dash_2_sol_2_a30<double>(), rk4_dash_2_sol_2_a31<double>()};
}

void check_rk42(const char *label, const rk42 &c) {
  const double b3 = 1.0 - (c.b0 + c.b1 + c.b2);
  const double a21 = c.c2 - c.a20;
  const double a32 = c.c3 - (c.a30 + c.a31);
  expect_eq(label, c.b0 + c.b1 + c.b2 + b3, 1.0);
  // The stage rows have to reproduce the nodes. These are the relations
  // solve.cxx uses to fill in the missing coefficients.
  expect_eq(label, c.a20 + a21, c.c2);
  expect_eq(label, c.a30 + c.a31 + a32, c.c3);
  if (!(b3 == b3) || !(a21 == a21) || !(a32 == a32)) {
    std::cerr << label << " derived coefficient is NaN\n";
    std::exit(1);
  }
}
} // namespace

int main() {
  const rk42 s1 = sol1();
  expect_eq("RK4-2(1) c2", s1.c2, 7.0 / 25.0);
  expect_eq("RK4-2(1) c3", s1.c3, -13.0 / 25.0);
  expect_eq("RK4-2(1) b0", s1.b0, -643.0 / 1536.0);
  expect_eq("RK4-2(1) b1", s1.b1, -4237.0 / 1092.0);
  expect_eq("RK4-2(1) b2", s1.b2, 38125.0 / 10752.0);
  expect_eq("RK4-2(1) a20", s1.a20, -49.0 / 1250.0);
  expect_eq("RK4-2(1) a30", s1.a30, 7033.0 / 960000.0);
  expect_eq("RK4-2(1) a31", s1.a31, -217633.0 / 210000.0);
  check_rk42("RK4-2(1)", s1);

  const rk42 s2 = sol2();
  expect_eq("RK4-2(2) c2", s2.c2, -99.0 / 50.0);
  expect_eq("RK4-2(2) c3", s2.c3, 101.0 / 100.0);
  expect_eq("RK4-2(2) b0", s2.b0, -191.0 / 882.0);
  expect_eq("RK4-2(2) b1", s2.b1, 48241.0 / 59994.0);
  expect_eq("RK4-2(2) b2", s2.b2, 193750.0 / 4351347.0);
  expect_eq("RK4-2(2) a20", s2.a20, 1309.0 / 15500.0);
  expect_eq("RK4-2(2) a30", s2.a30, -241289.0 / 5880000.0);
  expect_eq("RK4-2(2) a31", s2.a31, 22846301.0 / 16170000.0);
  check_rk42("RK4-2(2)", s2);

  expect_eq("RK4-3 c3", rk4_dash_3_sol_1_c3<double>(), 9.0 / 25.0);
  expect_eq("RK4-3 b0", rk4_dash_3_sol_1_b0<double>(), -85.0 / 1416.0);
  expect_eq("RK4-3 b1", rk4_dash_3_sol_1_b1<double>(), 131.0 / 408.0);
  expect_eq("RK4-3 b2", rk4_dash_3_sol_1_b2<double>(), -29.0 / 24.0);
  expect_eq("RK4-3 a30", rk4_dash_3_sol_1_a30<double>(), 2511.0 / 62500.0);
  expect_eq("RK4-3 a31", rk4_dash_3_sol_1_a31<double>(), -2268.0 / 15625.0);
  const double b0 = rk4_dash_3_sol_1_b0<double>();
  const double b1 = rk4_dash_3_sol_1_b1<double>();
  const double b2 = rk4_dash_3_sol_1_b2<double>();
  const double b3 = 1.0 - (b0 + b1 + b2);
  const double a30 = rk4_dash_3_sol_1_a30<double>();
  const double a31 = rk4_dash_3_sol_1_a31<double>();
  const double c3 = rk4_dash_3_sol_1_c3<double>();
  const double a32 = c3 - (a30 + a31);
  expect_eq("RK4-3 sum b", b0 + b1 + b2 + b3, 1.0);
  expect_eq("RK4-3 row", a30 + a31 + a32, c3);

  std::cout << "RK4-2 and RK4-3 coefficients match the published fractions\n";
  return 0;
}
