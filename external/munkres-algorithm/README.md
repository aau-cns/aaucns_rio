
# Munkres (Hungarian) Algorithm

Modern <header-only> C++ implementation of the Munkres (Hungarian) algorithm.

### Reference

Implementation based on [Dr Murray Pilgram](http://csclab.murraystate.edu/~bob.pilgrim/)'s tutorial paper [Tutorial on Implementation of Munkres' Assignment Algorithm](https://www.researchgate.net/publication/290437481_Tutorial_on_Implementation_of_Munkres'_Assignment_Algorithm).

There's an excellent discussion of the Munkres Algorithm at [topcoder.com](https://www.topcoder.com/community/competitive-programming/tutorials/assignment-problem-and-hungarian-algorithm/).

### Requirements

 * A [C++14 or C++17 compatible](https://en.cppreference.com/w/cpp/compiler_support) compiler
 * [GNU Make](https://www.gnu.org/software/make/) (To build and run examples.)

### Testcases

Tested on Ubuntu 18.04, with [Clang 6.0](http://releases.llvm.org/6.0.1/tools/clang/docs/ReleaseNotes.html) and [gcc 7.3.0](https://www.gnu.org/software/gcc/gcc-7/). Should work without issue on [Visual Studio](https://visualstudio.microsoft.com/), or any other compiler with [C++14 or C++17 support](https://en.cppreference.com/w/cpp/compiler_support).

### Example

The algorithm's interface abstracts away details of how the input graph is stored:

```c_cpp
//
// @param n_lhs_verts Number of left-hand-side vertices (in bipartite graph)
// @param n_rhs_verts Number of right-hand-side verices (ibid)
// @param cost Cost between vertices 'l' and 'r'. Use of function to abstract
//             away storage details of input graph.
// @see example.cpp
//
template<typename T>
std::vector<std::pair<unsigned, unsigned>> inline munkres_algorithm(
    const unsigned n_lhs_verts,
    const unsigned n_rhs_verts,
    std::function<T(unsigned l, unsigned r)> cost) noexcept;
```

This can be used as follows (see example.cpp)

```c_cpp

void some_function()
{
    std::vector<int> edge_weights = {40, 60, 15, 25, 30, 45, 55, 30, 25};
    auto f = [&] (unsigned r, unsigned c) { return edge_weights[r * 3 + c]; };
    auto matching = munkres_algorithm<int>(3, 3, f); // That simple.
}

```

