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

#include "aaucns_rio/fg/marginalization.h"

#include <gtsam/base/SymmetricBlockMatrix.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/GaussianFactorGraph.h>
#include <gtsam/nonlinear/LinearContainerFactor.h>

#include "aaucns_rio/debug.h"
namespace aaucns_rio
{
using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::P;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;

namespace
{
gtsam::FastVector<gtsam::Key> getPFKeys(
    const gtsam::FastVector<gtsam::Key>& keys)
{
    gtsam::FastVector<gtsam::Key> pf_keys;
    std::copy_if(keys.begin(), keys.end(), std::back_inserter(pf_keys),
                 [](const auto key) { return gtsam::symbolChr(key) == 'p'; });
    return pf_keys;
}
}  // namespace

std::pair<gtsam::Matrix, gtsam::Vector>
Marginalization::pairFromAugmentedHessian(const gtsam::Matrix& matrix)
{
    int n = matrix.rows() - 1;
    return std::pair<gtsam::Matrix, gtsam::Vector>(matrix.block(0, 0, n, n),
                                                   matrix.block(0, n, n, 1));
}

// Sort keys so that the order is - {b0 v0 x0 ... bn vn xn p0 ... pn}
gtsam::FastVector<gtsam::Key> Marginalization::sortKeys(
    const gtsam::FastSet<gtsam::Key>& set_to_sort)
{
    // Allocate separate vectors for x, v, b, p. Then merge.
    gtsam::FastVector<gtsam::Key> x;
    std::copy_if(set_to_sort.begin(), set_to_sort.end(), std::back_inserter(x),
                 [](const auto key) { return gtsam::symbolChr(key) == 'x'; });
    std::sort(x.begin(), x.end(), [](const auto i, const auto j) {
        return gtsam::symbolIndex(i) < gtsam::symbolIndex(j);
    });

    gtsam::FastVector<gtsam::Key> v;
    std::copy_if(set_to_sort.begin(), set_to_sort.end(), std::back_inserter(v),
                 [](const auto key) { return gtsam::symbolChr(key) == 'v'; });
    std::sort(v.begin(), v.end(), [](const auto i, const auto j) {
        return gtsam::symbolIndex(i) < gtsam::symbolIndex(j);
    });

    gtsam::FastVector<gtsam::Key> b;
    std::copy_if(set_to_sort.begin(), set_to_sort.end(), std::back_inserter(b),
                 [](const auto key) { return gtsam::symbolChr(key) == 'b'; });
    std::sort(b.begin(), b.end(), [](const auto i, const auto j) {
        return gtsam::symbolIndex(i) < gtsam::symbolIndex(j);
    });

    gtsam::FastVector<gtsam::Key> nav_states_sorted;
    for (int i = 0; i < x.size(); ++i)
    {
        nav_states_sorted.push_back(x[i]);
        nav_states_sorted.push_back(v[i]);
        nav_states_sorted.push_back(b[i]);
    }

    gtsam::FastVector<gtsam::Key> p;
    std::copy_if(set_to_sort.begin(), set_to_sort.end(), std::back_inserter(p),
                 [](const auto key) { return gtsam::symbolChr(key) == 'p'; });
    std::sort(p.begin(), p.end(), [](const auto i, const auto j) {
        return gtsam::symbolIndex(i) < gtsam::symbolIndex(j);
    });

    nav_states_sorted.insert(nav_states_sorted.end(), p.begin(), p.end());
    return nav_states_sorted;
}

// Decrement nav states indices so that they correspond to the indices after the
// shift.
gtsam::FastVector<gtsam::Key> Marginalization::decrementNavStatesKeys(
    const gtsam::FastVector<gtsam::Key>& keys)
{
    gtsam::FastVector<gtsam::Key> decremented_keys;
    std::for_each(keys.begin(), keys.end(), [&](const auto& key) {
        if (gtsam::symbolChr(key) == 'x' || gtsam::symbolChr(key) == 'v' ||
            gtsam::symbolChr(key) == 'b')
        {
            decremented_keys.push_back(gtsam::Symbol(
                gtsam::symbolChr(key), gtsam::symbolIndex(key) - 1));
        }
        else
        {
            decremented_keys.push_back(key);
        }
    });
    return decremented_keys;
}

gtsam::NonlinearFactorGraph Marginalization::marginalizeOut(
    const gtsam::NonlinearFactorGraph& graph,
    const gtsam::Values& values_after_shift, const gtsam::Values& solution,
    const gtsam::FastVector<gtsam::Key>& keysToMarginalize)
{
    gtsam::NonlinearFactorGraph marginalizedOutGraph;

    gtsam::FastSet<gtsam::Key> setOfKeysToMarginalize(keysToMarginalize);
    gtsam::FastSet<gtsam::Key> connectedKeys;

    extractKeysToMarginalize(graph, marginalizedOutGraph,
                             setOfKeysToMarginalize, connectedKeys);

    gtsam::GaussianFactorGraph::shared_ptr linearizedFactorsToMarginalize =
        marginalizedOutGraph.linearize(solution);
    std::map<gtsam::Key, size_t> keyDimMap =
        linearizedFactorsToMarginalize->getKeyDimMap();

    int mSize = 0;
    int aSize = 0;

    // Ordering of all variables participating in the marginalization.
    gtsam::Ordering ordering;
    // Ordering of only connected variables participating in the
    // marginalization.
    gtsam::Ordering connectedOrdering;

    // Here sort `setOfKeysToMarginalize` and `connectedKeys`.
    const gtsam::FastVector<gtsam::Key> sorted_keys_to_marginalize =
        sortKeys(setOfKeysToMarginalize);
    sorted_connected_keys_ = sortKeys(connectedKeys);

    gtsam::FastVector<size_t> connectedDims;
    for (const gtsam::Key& k : sorted_keys_to_marginalize)
    {
        ordering.push_back(k);
        mSize += keyDimMap[k];
    }
    for (const gtsam::Key& k : sorted_connected_keys_)
    {
        ordering.push_back(k);
        connectedOrdering.push_back(k);
        connectedDims.push_back(keyDimMap[k]);
        aSize += keyDimMap[k];
    }

    gtsam::Matrix hessian =
        linearizedFactorsToMarginalize->augmentedHessian(ordering);
    gtsam::Matrix HAfterSchurComplement =
        computeSchurComplement(hessian, mSize, aSize);

    gtsam::SymmetricBlockMatrix sm(connectedDims, true);
    sm.setFullMatrix(HAfterSchurComplement);

    // Make new ordering like `connectedOrdering` but decrement indexes on all
    // variables except PFs.
    const gtsam::FastVector<gtsam::Key> decremented_sorted_connected_keys =
        sortKeys(gtsam::FastSet<gtsam::Key>(
            decrementNavStatesKeys(sorted_connected_keys_)));

    const gtsam::Values all_values_after_shift = buildValuesForLinearization(
        values_after_shift, solution, sorted_keys_to_marginalize);

    gtsam::NonlinearFactorGraph newGraph;
    gtsam::LinearContainerFactor lcf(
        gtsam::HessianFactor(decremented_sorted_connected_keys, sm),
        all_values_after_shift);
    newGraph.add(lcf);

    return newGraph;
}

gtsam::Values Marginalization::buildValuesForLinearization(
    const gtsam::Values& values_after_shift, const gtsam::Values& solution,
    const gtsam::FastVector<gtsam::Key>& keys_to_marginalize) const
{
    gtsam::Values all_values_after_shift = values_after_shift;
    // Place PF keys from solution into `values_after_shift` .
    for (const auto& key : getPFKeysInPrior())
    {
        all_values_after_shift.insert(key, solution.at<gtsam::Point3>(key));
    }
    return all_values_after_shift;
}

void Marginalization::extractKeysToMarginalize(
    const gtsam::NonlinearFactorGraph& graph,
    gtsam::NonlinearFactorGraph& marginalizedOutGraph,
    gtsam::FastSet<gtsam::Key>& setOfKeysToMarginalize,
    gtsam::FastSet<gtsam::Key>& connectedKeys)
{
    for (size_t i = 0; i < graph.size(); i++)
    {
        gtsam::NonlinearFactor::shared_ptr factor = graph.at(i);
        gtsam::FastSet<gtsam::Key> set_of_factor_keys(factor->keys());
        gtsam::FastSet<gtsam::Key> intersection;

        // intersection.
        std::set_intersection(
            setOfKeysToMarginalize.begin(), setOfKeysToMarginalize.end(),
            set_of_factor_keys.begin(), set_of_factor_keys.end(),
            std::inserter(intersection, intersection.begin()));

        if (!intersection.empty())
        {
            // Place keys connected to the keys to be marginalized in the
            // `connectedKeys`.
            std::set_difference(
                set_of_factor_keys.begin(), set_of_factor_keys.end(),
                setOfKeysToMarginalize.begin(), setOfKeysToMarginalize.end(),
                std::inserter(connectedKeys, connectedKeys.begin()));
            // Add factor which has keys to marginalize in the graph.
            marginalizedOutGraph.add(factor);
        }
    }
}

bool Marginalization::isPFKeyInPrior(const gtsam::Key key) const
{
    gtsam::FastVector<gtsam::Key> pfs_in_prior = getPFKeysInPrior();
    return (std::find(pfs_in_prior.begin(), pfs_in_prior.end(), key) !=
            pfs_in_prior.end());
}

gtsam::FastVector<gtsam::Key> Marginalization::getPFKeysInPrior() const
{
    return getPFKeys(sorted_connected_keys_);
}

gtsam::FastVector<gtsam::Key> Marginalization::getPFKeysToMarginalize(
    const CircularBuffer<RadarPFDistanceFactorData, ConfigFg::kNPosesInWindow>&
        pf_factor_data_horizon)
{
    // Get indexes of PFs in the oldest dataslots x0 and x1.
    // Then find which PFs are in x0 but are not anymore in x1.
    // These are marginalized out as they disappear.
    std::set<std::size_t> pfs_ids_x0;
    std::for_each(pf_factor_data_horizon[0]
                      .most_recent_updated_state_.persistent_features_.begin(),
                  pf_factor_data_horizon[0]
                      .most_recent_updated_state_.persistent_features_.end(),
                  [&](const auto pf) { pfs_ids_x0.insert(pf.id); }

    );
    std::set<std::size_t> pfs_ids_x1;
    std::for_each(pf_factor_data_horizon[1]
                      .most_recent_updated_state_.persistent_features_.begin(),
                  pf_factor_data_horizon[1]
                      .most_recent_updated_state_.persistent_features_.end(),
                  [&](const auto pf) { pfs_ids_x1.insert(pf.id); });

    std::vector<std::size_t> pfs_ids_to_marginalize;
    std::set_difference(
        pfs_ids_x0.begin(), pfs_ids_x0.end(), pfs_ids_x1.begin(),
        pfs_ids_x1.end(),
        std::inserter(pfs_ids_to_marginalize, pfs_ids_to_marginalize.begin()));
    gtsam::FastVector<gtsam::Key> pfs_keys_to_marginalize;
    std::for_each(
        pfs_ids_to_marginalize.begin(), pfs_ids_to_marginalize.end(),
        [&](const auto id) { pfs_keys_to_marginalize.push_back(P(id)); });
    return pfs_keys_to_marginalize;
}

gtsam::Matrix Marginalization::computeSchurComplement(
    const gtsam::Matrix& augmentedHessian, int mSize, int aSize)
{
    auto pair = pairFromAugmentedHessian(augmentedHessian);

    gtsam::Vector SVec = (pair.first.diagonal().cwiseAbs() +
                          gtsam::Vector::Constant(pair.first.cols(), 10))
                             .cwiseSqrt();
    gtsam::Vector SVecI = SVec.cwiseInverse();

    gtsam::Matrix hessianScaled =
        SVecI.asDiagonal() * pair.first * SVecI.asDiagonal();
    gtsam::Vector bScaled = SVecI.asDiagonal() * pair.second;

    gtsam::Matrix Hmm = hessianScaled.block(0, 0, mSize, mSize);
    gtsam::Matrix Hma = hessianScaled.block(0, mSize, mSize, aSize);
    gtsam::Matrix Haa = hessianScaled.block(mSize, mSize, aSize, aSize);

    gtsam::Vector bm = bScaled.segment(0, mSize);
    gtsam::Vector ba = bScaled.segment(mSize, aSize);

    // Compute inverse.
    gtsam::Matrix HmmInv =
        Hmm.completeOrthogonalDecomposition().pseudoInverse();

    gtsam::Matrix HaaNew = Haa - Hma.transpose() * HmmInv * Hma;
    gtsam::Vector baNew = ba - Hma.transpose() * HmmInv * bm;

    // Unscale
    gtsam::Vector SVecUpdated = SVec.segment(mSize, aSize);
    gtsam::Matrix HNewUnscaled =
        SVecUpdated.asDiagonal() * HaaNew * SVecUpdated.asDiagonal();
    gtsam::Matrix bNewUnscaled = SVecUpdated.asDiagonal() * baNew;

    // Make Hessian symmetric for numeric reasons.
    HNewUnscaled = 0.5 * (HNewUnscaled.transpose() + HNewUnscaled).eval();

    gtsam::Matrix augmentedHRes(aSize + 1, aSize + 1);
    augmentedHRes.setZero();
    augmentedHRes.topLeftCorner(aSize, aSize) = HNewUnscaled;
    augmentedHRes.topRightCorner(aSize, 1) = bNewUnscaled;
    augmentedHRes.bottomLeftCorner(1, aSize) = bNewUnscaled.transpose();

    return augmentedHRes;
}
}  // namespace aaucns_rio