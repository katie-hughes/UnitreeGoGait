#include "gaitlib/gaits.hpp"
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

namespace gaitlib
{

    TEST_CASE("Factorial", "[gaits]")
    {
        REQUIRE(factorial(0) == 1);
        REQUIRE(factorial(1) == 1);
        REQUIRE(factorial(2) == 2);
        REQUIRE(factorial(3) == 6);
        REQUIRE(factorial(4) == 24);
        REQUIRE(factorial(5) == 120);
    }

    TEST_CASE("Choose", "[gaits]")
    {
        REQUIRE(choose(0,0) == 1);

        REQUIRE(choose(1,0) == 1);
        REQUIRE(choose(1,1) == 1);

        REQUIRE(choose(2,0) == 1);
        REQUIRE(choose(2,1) == 2);
        REQUIRE(choose(2,2) == 1);

        REQUIRE(choose(3,0) == 1);
        REQUIRE(choose(3,1) == 3);
        REQUIRE(choose(3,2) == 3);
        REQUIRE(choose(3,3) == 1);

        REQUIRE(choose(4,0) == 1);
        REQUIRE(choose(4,1) == 4);
        REQUIRE(choose(4,2) == 6);
        REQUIRE(choose(4,3) == 4);
        REQUIRE(choose(4,4) == 1);

        REQUIRE(choose(5,0) == 1);
        REQUIRE(choose(5,1) == 5);
        REQUIRE(choose(5,2) == 10);
        REQUIRE(choose(5,3) == 10);
        REQUIRE(choose(5,4) == 5);
        REQUIRE(choose(5,5) == 1);
    }

    TEST_CASE("Simple Modulate", "[gaits]")
    {
        std::vector<double> init = {1.0, 2.0, 3.0, 4.0};
        std::vector<double> mod_25 = {2.0, 3.0, 4.0, 1.0};
        std::vector<double> mod_50 = {3.0, 4.0, 1.0, 2.0};
        std::vector<double> mod_75 = {4.0, 1.0, 2.0, 3.0};
        REQUIRE(modulate(init, 0.25) == mod_25);
        REQUIRE(modulate(init, 0.50) == mod_50);
        REQUIRE(modulate(init, 0.75) == mod_75);
        REQUIRE(modulate(init, 0.00) == init);
        REQUIRE(modulate(init, 1.00) == init);
    }

    TEST_CASE("ik3D: Z=0", "[gaits]")
    {
        // first test that it's the same as regular ik if z = 0
        const auto x = 0.0;
        const auto y = -0.1;
        const auto ik_regular = ik(x, y);
        const auto ik_3d = ik3D(x, y, 0.0);
        REQUIRE(ik_3d.hip == 0.0);
        REQUIRE(ik_3d.leg.calf_lefty  == ik_regular.calf_lefty);
        REQUIRE(ik_3d.leg.calf_righty == ik_regular.calf_righty);
        REQUIRE(ik_3d.leg.thigh_lefty  == ik_regular.thigh_lefty);
        REQUIRE(ik_3d.leg.thigh_righty == ik_regular.thigh_righty);
    }

}