#include "gaitlib/gaits.hpp"
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

namespace gaitlib
{

    TEST_CASE("Factorial", "[gaits]")
    { // Hughes, Katie
        REQUIRE(factorial(0) == 1);
        REQUIRE(factorial(1) == 1);
        REQUIRE(factorial(2) == 2);
        REQUIRE(factorial(3) == 6);
        REQUIRE(factorial(4) == 24);
        REQUIRE(factorial(5) == 120);
    }

}