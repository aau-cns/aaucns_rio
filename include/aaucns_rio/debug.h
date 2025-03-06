// Copyright (C) 2024 Jan Michalczyk, Control of Networked Systems, University
// of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <jan.michalczyk@aau.at>

#ifndef _DEBUG_H_
#define _DEBUG_H_

#include <Eigen/Dense>

// Below needed for munkres printout - remove after debug
#include <cstdlib>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <set>
#include <sstream>
#include <string>
// Above needed for munkres printout - remove after debug

namespace aaucns_rio
{
namespace debug
{
//---------------------------------------
// ------------------------------------------------------------- assignment cost
//
template <typename T>
static T assignment_cost(
    std::function<T(unsigned l, unsigned r)> cost_func,
    const std::vector<std::pair<unsigned, unsigned>>& matching) noexcept
{
    T cost = T(0);
    for (const auto& m : matching) cost += cost_func(m.first, m.second);
    return cost;
}

// -------------------------------------------------------- print munkres result
//
static std::string print_munkres_result(
    const unsigned n_lhs_verts, const unsigned n_rhs_verts,
    std::function<double(unsigned l, unsigned r)> cost_func,
    const std::vector<std::pair<unsigned, unsigned>>& matching) noexcept
{
    static const char* ANSI_COLOUR_BLUE_BG = "\x1b[44m";
    static const char* ANSI_COLOUR_RESET = "\x1b[0m";

    std::stringstream ss("");

    auto is_matching = [&](const unsigned r, const unsigned c) {
        const auto ii = std::find_if(
            begin(matching), end(matching),
            [&](const auto& x) { return (x.first == r and x.second == c); });
        return ii != end(matching);
    };

    ss << std::setprecision(4);
    ss << "|L|  = " << n_lhs_verts << std::endl;
    ss << "|R|  = " << n_rhs_verts << std::endl;
    ss << "cost = " << assignment_cost(cost_func, matching) << std::endl;
    for (auto r = 0u; r < n_lhs_verts; ++r)
    {
        for (auto c = 0u; c < n_rhs_verts; ++c)
        {
            if (c > 0) ss << "  ";
            const bool it_is = is_matching(r, c);
            if (it_is) ss << ANSI_COLOUR_BLUE_BG;
            ss << cost_func(r, c);
            if (it_is) ss << ANSI_COLOUR_RESET;
        }
        ss << std::endl;
    }

    return ss.str();
}
//---------------------------------------

template <typename t1, typename t2>
std::ostream& operator<<(std::ostream& os, const std::pair<t1, t2>& p)
{
    os << "first: " << p.first << " "
       << "second: " << p.second << std::endl;
    return os;
}

class Logger
{
   public:
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

    Logger(const bool dump_log) : dump_log_(dump_log)
    {
        if (dump_log_)
        {
            logfile_.open("aaucns_rio_log.json");
            if (logfile_.is_open())
            {
                logfile_ << "{"
                         << "\n";
                logfile_ << "\"log\": ["
                         << "\n";
            }
        }
    }

    void startLoggingIteration()
    {
        if (dump_log_)
        {
            if (logfile_.is_open())
            {
                logfile_ << "{"
                         << "\n";
                logfile_ << "\"iteration\": ["
                         << "\n";
            }
        }
    }

    void stopLoggingIteration()
    {
        if (dump_log_)
        {
            if (logfile_.is_open())
            {
                terminateJSONEntryWithComma();
            }
        }
    }

    ~Logger()
    {
        if (dump_log_)
        {
            if (logfile_.is_open())
            {
                terminateJSONEntry();
                logfile_.close();
            }
        }
    }

    template <typename t>
    void writeEigenMatrixToFile(const Eigen::MatrixBase<t>& matrix,
                                const std::string& entry_name)
    {
        if (dump_log_)
        {
            const static Eigen::IOFormat MatrixFormat(
                Eigen::FullPrecision, 0, ", ", ",\n", "[", "]", "[", "]");
            const static Eigen::IOFormat ArrayFormat(
                Eigen::FullPrecision, 0, ", ", ",\n", "[", "]", "[", "]");
            if (logfile_.is_open())
            {
                logfile_ << "{"
                         << "\n";
                logfile_ << "\"" << entry_name << "\": \n";
                if (matrix.cols() == 1 || matrix.rows() == 1)
                {
                    logfile_ << matrix.format(ArrayFormat) << "\n";
                }
                else
                {
                    logfile_ << matrix.format(MatrixFormat) << "\n";
                }
                logfile_ << "},"
                         << "\n";
            }
        }
    }

    template <typename t>
    static void printMatrix(const Eigen::MatrixBase<t>& matrix)
    {
        if (matrix.size() > 0)
        {
            for (int i = 0; i < matrix.rows(); ++i)
            {
                for (int j = 0; j < matrix.cols(); ++j)
                {
                    std::cerr << matrix(i, j) << " ";
                }
                std::cerr << std::endl;
            }
        }
    }

    template <typename t>
    static void printMatrix(const Eigen::MatrixBase<t>& matrix,
                            const std::string& message)
    {
        std::cout << "------ begin " << message << "------" << std::endl;
        if (matrix.size() > 0)
        {
            for (int i = 0; i < matrix.rows(); ++i)
            {
                for (int j = 0; j < matrix.cols(); ++j)
                {
                    std::cerr << matrix(i, j) << " ";
                }
                std::cerr << std::endl;
            }
        }
        std::cout << "------ end " << message << "------" << std::endl;
    }

    template <typename t>
    static void printStdVector(const std::vector<std::vector<t>>& vector)
    {
        for (const auto& inner_vector : vector)
        {
            for (const auto& inner_vector_item : inner_vector)
            {
                std::cerr << inner_vector_item << " ";
            }
            std::cerr << std::endl;
        }
    }

    template <typename t>
    static void printStdVector(const std::vector<t>& vector)
    {
        for (const auto& vector_item : vector)
        {
            std::cerr << vector_item << " ";
        }
        std::cerr << std::endl;
    }

    template <typename t>
    static void printStdSet(const std::set<t>& set)
    {
        for (const auto& set_item : set)
        {
            std::cerr << set_item << " ";
        }
        std::cerr << std::endl;
    }

    template <typename t>
    static void printVariable(const t variable, const std::string& message)
    {
        std::cout << "------ begin " << message << "------" << std::endl;
        std::cout << variable << std::endl;
        std::cout << "------ end " << message << "------" << std::endl;
    }

    static void printVariable(const uint8_t variable,
                              const std::string& message)
    {
        std::cout << "------ begin " << message << "------" << std::endl;
        std::cout << static_cast<int>(variable) << std::endl;
        std::cout << "------ end " << message << "------" << std::endl;
    }

   private:
    bool dump_log_;
    std::ofstream logfile_;

    void terminateJSONEntry()
    {
        // Remove the "," and "\n" at the end of the file to conform to
        // JSON.
        const auto pos = logfile_.tellp();
        constexpr std::size_t kNCharsToDeleteFromEnd = 2;
        logfile_.seekp(pos - kNCharsToDeleteFromEnd);
        logfile_ << "\n";
        logfile_ << "]\n}"
                 << "\n";
    }

    void terminateJSONEntryWithComma()
    {
        // Remove the "," and "\n" at the end of the file to conform to
        // JSON.
        const auto pos = logfile_.tellp();
        constexpr std::size_t kNCharsToDeleteFromEnd = 2;
        logfile_.seekp(pos - kNCharsToDeleteFromEnd);
        logfile_ << "]\n},"
                 << "\n";
    }
};

}  // namespace debug
}  // namespace aaucns_rio

#endif /* _DEBUG_H_ */
