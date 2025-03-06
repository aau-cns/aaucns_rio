
#include <algorithm>
#include <iostream>
#include <numeric>
#include <random>
#include <vector>

#include "munkres.hpp"

#define CATCH_CONFIG_PREFIX_ALL
#define CATCH_CONFIG_RUNNER
#include "catch.hpp"

// ------------------------------------------------------------- assignment cost
//
template<typename T>
static T assignment_cost(
    std::function<T(unsigned l, unsigned r)> cost_func,
    const std::vector<std::pair<unsigned, unsigned>>& matching) noexcept
{
   T cost = 0.0;
   for(const auto& m : matching) cost += cost_func(m.first, m.second);
   return cost;
}

// ------------------------------------------------------------- naive hungarian
// O(n!)
template<typename T>
static std::vector<std::pair<unsigned, unsigned>>
naive_hungarian(const unsigned n_lhs_verts,
                const unsigned n_rhs_verts,
                std::function<T(unsigned l, unsigned r)> cost_func) noexcept
{
   //
   const bool is_l_assigned = n_lhs_verts >= n_rhs_verts;
   const unsigned sz        = std::max(n_lhs_verts, n_rhs_verts);
   std::vector<unsigned> assignments(sz);
   std::iota(begin(assignments), end(assignments), 0);

   auto make_assignment = [&](const std::vector<unsigned>& assignments) {
      std::vector<std::pair<unsigned, unsigned>> o;
      o.reserve(sz);
      if(is_l_assigned) {
         for(auto i = 0u; i < sz; ++i)
            if(assignments[i] < n_rhs_verts) o.push_back({i, assignments[i]});
      } else {
         for(auto i = 0u; i < sz; ++i)
            if(assignments[i] < n_lhs_verts) o.push_back({assignments[i], i});
      }
      return o;
   };

   auto best      = make_assignment(assignments);
   auto best_cost = assignment_cost(cost_func, best);

   while(std::next_permutation(begin(assignments), end(assignments))) {
      const auto matching = make_assignment(assignments);
      const auto cost     = assignment_cost(cost_func, matching);
      if(cost < best_cost) {
         best      = matching;
         best_cost = cost;
      }
   }

   return best;
}

// ------------------------------------------------------------------- TEST_CASE
//
CATCH_TEST_CASE("MunkresAlgorithm", "[munkres-algorithm]")
{
   std::mt19937 g;
   g.seed(1);

   std::uniform_int_distribution<int> distribution{1, 5};
   std::uniform_int_distribution<int> random_M{-9999, 9999};

   CATCH_SECTION("munkres-algorithm")
   {
      auto test_it = [&](const bool is_degenerate) {
         const auto n_rows = distribution(g);
         const auto n_cols = distribution(g);
         std::vector<double> M(n_rows * n_cols);
         std::generate(begin(M), end(M), [&]() {
            return is_degenerate
                       ? double(distribution(g) % 2 + 1) // [1..2]
                       : double(random_M(g)) / 1000.0;   // [-9.99..9.99]
         });
         auto f = [&](unsigned r, unsigned c) { return M[r * n_cols + c]; };

         const auto matching0 = naive_hungarian<double>(n_rows, n_cols, f);
         const auto matching1 = munkres_algorithm<double>(n_rows, n_cols, f);
         const auto cost0     = assignment_cost<double>(f, matching0);
         const auto cost1     = assignment_cost<double>(f, matching1);
         CATCH_REQUIRE(std::fabs(cost0 - cost1) < 1e20);
      };

      for(auto i = 0; i < 1000; ++i) test_it(true); // test degenerate cases
      for(auto i = 0; i < 1000; ++i) test_it(false);
   }
}

// ------------------------------------------------------------------------ main
//
int main(int argc, char** argv)
{
   Catch::Session session; // There must be exactly one instance

   // Let Catch (using Clara) parse the command line
   auto return_code = session.applyCommandLine(argc, argv);
   if(return_code != EXIT_SUCCESS) return return_code; // Command line error

   return session.run();
}
