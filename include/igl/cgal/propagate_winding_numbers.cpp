// This file is part of libigl, a simple c++ geometry processing library.
// 
// Copyright (C) 2015 Qingnan Zhou <qnzhou@gmail.com>
// 
// This Source Code Form is subject to the terms of the Mozilla Public License 
// v. 2.0. If a copy of the MPL was not distributed with this file, You can 
// obtain one at http://mozilla.org/MPL/2.0/.
//
#include "propagate_winding_numbers.h"
#include "../extract_manifold_patches.h"
#include "../extract_non_manifold_edge_curves.h"
#include "../facet_components.h"
#include "../unique_edge_map.h"
#include "../writeOBJ.h"
#include "../writePLY.h"
#include "order_facets_around_edge.h"
#include "outer_facet.h"
#include "closest_facet.h"
#include "assign_scalar.h"
#include "extract_cells.h"

#include <stdexcept>
#include <limits>
#include <vector>
#include <tuple>
#include <queue>

namespace propagate_winding_numbers_helper {
    template<typename DerivedW >
    bool winding_number_assignment_is_consistent(
            const std::vector<Eigen::VectorXi>& orders,
            const std::vector<std::vector<bool> >& orientations,
            const Eigen::PlainObjectBase<DerivedW>& per_patch_winding_number) {
        const size_t num_edge_curves = orders.size();
        const size_t num_labels = per_patch_winding_number.cols() / 2;
        for (size_t i=0; i<num_edge_curves; i++) {
            const auto& order = orders[i];
            const auto& orientation = orientations[i];
            assert(order.size() == orientation.size());
            const size_t order_size = order.size();
            for (size_t j=0; j<order_size; j++) {
                const size_t curr = j;
                const size_t next = (j+1) % order_size;

                for (size_t k=0; k<num_labels; k++) {
                    // Retrieve the winding numbers of the current partition from
                    // the current patch and the next patch.  If the patches forms
                    // the boundary of a 3D volume, the winding number assignments
                    // should be consistent.
                    int curr_winding_number = per_patch_winding_number(
                            order[curr], k*2+(orientation[curr]? 0:1));
                    int next_winding_number = per_patch_winding_number(
                            order[next], k*2+(orientation[next]? 1:0));
                    if (curr_winding_number != next_winding_number) {
                        std::cout << "edge: " << i << std::endl;
                        std::cout << curr_winding_number << " != " <<
                            next_winding_number << std::endl;
                        return false;
                    }
                }
            }
        }
        return true;
    }

    template<
        typename DerivedF,
        typename DeriveduE,
        typename uE2EType >
    bool is_orientable(
            const Eigen::PlainObjectBase<DerivedF>& F,
            const Eigen::PlainObjectBase<DeriveduE>& uE,
            const std::vector<std::vector<uE2EType> >& uE2E) {
        const size_t num_faces = F.rows();
        const size_t num_edges = uE.rows();
        auto edge_index_to_face_index = [&](size_t ei) {
            return ei % num_faces;
        };
        auto is_consistent = [&](size_t fid, size_t s, size_t d) {
            if (F(fid, 0) == s && F(fid, 1) == d) return true;
            if (F(fid, 1) == s && F(fid, 2) == d) return true;
            if (F(fid, 2) == s && F(fid, 0) == d) return true;

            if (F(fid, 0) == d && F(fid, 1) == s) return false;
            if (F(fid, 1) == d && F(fid, 2) == s) return false;
            if (F(fid, 2) == d && F(fid, 0) == s) return false;
            throw "Invalid face!!";
        };
        for (size_t i=0; i<num_edges; i++) {
            const size_t s = uE(i,0);
            const size_t d = uE(i,1);
            int count=0;
            for (const auto& ei : uE2E[i]) {
                const size_t fid = edge_index_to_face_index(ei);
                if (is_consistent(fid, s, d)) count++;
                else count--;
            }
            if (count != 0) {
                return false;
            }
        }
        return true;
    }
}

template<
    typename DerivedV,
    typename DerivedF,
    typename DeriveduE,
    typename uE2EType,
    typename DerivedC,
    typename DerivedP,
    typename DerivedW >
IGL_INLINE bool igl::cgal::propagate_winding_numbers_single_component_patch_wise(
        const Eigen::PlainObjectBase<DerivedV>& V,
        const Eigen::PlainObjectBase<DerivedF>& F,
        const Eigen::PlainObjectBase<DeriveduE>& uE,
        const std::vector<std::vector<uE2EType> >& uE2E,
        const Eigen::PlainObjectBase<DerivedC>& labels,
        const Eigen::PlainObjectBase<DerivedP>& P,
        const std::vector<std::vector<size_t> >& intersection_curves,
        Eigen::PlainObjectBase<DerivedW>& patch_W) {
    const size_t num_faces = F.rows();
    const size_t num_patches = P.maxCoeff() + 1;
    assert(labels.size() == num_patches);
    // Utility functions.
    auto edge_index_to_face_index = [&](size_t ei) { return ei % num_faces; };
    auto edge_index_to_corner_index = [&](size_t ei) { return ei / num_faces; };
    auto face_and_corner_index_to_edge_index = [&](size_t fi, size_t ci) {
        return ci*num_faces + fi;
    };
    auto get_edge_end_points = [&](size_t ei, size_t& s, size_t& d) {
        const size_t fi = edge_index_to_face_index(ei);
        const size_t ci = edge_index_to_corner_index(ei);
        s = F(fi, (ci+1)%3);
        d = F(fi, (ci+2)%3);
    };
    auto is_positively_orientated =
        [&](size_t fi, size_t s, size_t d) -> bool{
            const Eigen::Vector3i f = F.row(fi);
            if (f[0] == d && f[1] == s) return true;
            if (f[1] == d && f[2] == s) return true;
            if (f[2] == d && f[0] == s) return true;
            if (f[0] == s && f[1] == d) return false;
            if (f[1] == s && f[2] == d) return false;
            if (f[2] == s && f[0] == d) return false;
            throw std::runtime_error("Edge does not belong to face.");
            return false;
        };

    auto compute_signed_index = [&](size_t fi, size_t s, size_t d) -> int{
        return int(fi+1) * (is_positively_orientated(fi, s, d) ? 1:-1);
    };
    auto compute_unsigned_index = [&](int signed_idx) -> size_t {
        return abs(signed_idx) - 1;
    };

    // Order patches around each intersection curve.
    const size_t num_edge_curves = intersection_curves.size();
    std::vector<Eigen::VectorXi> orders(num_edge_curves);
    std::vector<std::vector<bool> > orientations(num_edge_curves);
    std::vector<std::vector<size_t> > patch_curve_adjacency(num_patches);
    for (size_t i=0; i<num_edge_curves; i++) {
        const auto& curve = intersection_curves[i];
        const size_t uei = curve[0];
        size_t s = uE(uei, 0);
        size_t d = uE(uei, 1);
        std::vector<int> adj_faces;
        for (size_t ei : uE2E[uei]) {
            const size_t fi = edge_index_to_face_index(ei);
            const size_t signed_fi =
                compute_signed_index(fi, s, d);
            adj_faces.push_back(signed_fi);
            patch_curve_adjacency[P[fi]].push_back(i);
        }

        auto& order = orders[i];
        igl::cgal::order_facets_around_edge(
                V, F, s, d, adj_faces, order);
        assert(order.minCoeff() == 0);
        assert(order.maxCoeff() == adj_faces.size() - 1);

        auto& orientation = orientations[i];
        orientation.resize(order.size());
        std::transform(order.data(), order.data()+order.size(),
                orientation.begin(), 
                [&](size_t index) { return adj_faces[index] > 0; });
        std::transform(order.data(), order.data()+order.size(),
                order.data(), [&](size_t index) {
                return P[compute_unsigned_index(adj_faces[index])];
                } );
    }

    // Propagate winding number from infinity.
    // Assuming infinity has winding number 0.
    const size_t num_labels = labels.maxCoeff() + 1;
    const int INVALID = std::numeric_limits<int>::max();
    patch_W.resize(num_patches, 2*num_labels);
    patch_W.setConstant(INVALID);

    size_t outer_facet_idx;
    bool outer_facet_is_flipped;
    Eigen::VectorXi face_indices(num_faces);
    face_indices.setLinSpaced(num_faces, 0, num_faces-1);
    igl::cgal::outer_facet(V, F, face_indices,
            outer_facet_idx, outer_facet_is_flipped);
    size_t outer_patch_idx = P[outer_facet_idx];
    size_t outer_patch_label = labels[outer_patch_idx];
    patch_W.row(outer_patch_idx).setZero();
    if (outer_facet_is_flipped) {
        patch_W(outer_patch_idx, outer_patch_label*2) = -1;
    } else {
        patch_W(outer_patch_idx, outer_patch_label*2+1) = 1;
    }

    auto winding_num_assigned = [&](size_t patch_idx) -> bool{
        return (patch_W.row(patch_idx).array() != INVALID).all();
    };

    std::queue<size_t> Q;
    Q.push(outer_patch_idx);
    while (!Q.empty()) {
        const size_t curr_patch_idx = Q.front();
        const size_t curr_patch_label = labels[curr_patch_idx];
        Q.pop();

        const auto& adj_curves = patch_curve_adjacency[curr_patch_idx];
        for (size_t curve_idx : adj_curves) {
            const auto& order = orders[curve_idx];
            const auto& orientation = orientations[curve_idx];
            const size_t num_adj_patches = order.size();
            assert(num_adj_patches == orientation.size());

            size_t curr_i = std::numeric_limits<size_t>::max();
            for (size_t i=0; i<num_adj_patches; i++) {
                if (order[i] == curr_patch_idx) {
                    curr_i = i;
                    break;
                }
            }
            assert(curr_i < num_adj_patches);
            const bool curr_ori = orientation[curr_i];

            //for (size_t i=0; i<num_adj_patches; i++) {
            //    const size_t next_i = (curr_i + 1) % num_adj_patches;
            //    const size_t num_patch_idx = order[next_i];
            //    const bool next_ori = orientation[next_i];
            //    const size_t next_patch_label = labels[next_patch_idx];
            //    // TODO
            //}

            const size_t next_i = curr_ori ? (curr_i+1) % num_adj_patches
                : (curr_i+num_adj_patches-1)%num_adj_patches;
            const size_t prev_i = curr_ori ?
                (curr_i+num_adj_patches-1)%num_adj_patches
                : (curr_i+1) % num_adj_patches;
            const size_t next_patch_idx = order[next_i];
            const size_t prev_patch_idx = order[prev_i];

            if (!winding_num_assigned(next_patch_idx)) {
                const bool next_ori = orientation[next_i];
                const bool next_cons = next_ori != curr_ori;
                const size_t next_patch_label = labels[next_patch_idx];
                for (size_t i=0; i<num_labels; i++) {
                    const int shared_winding_number =
                        patch_W(curr_patch_idx, i*2);

                    if (i == next_patch_label) {
                        // Truth table:
                        // curr_ori  next_ori  wind_# inc
                        // True      True      -1
                        // True      False      1
                        // False     True       1
                        // False     False     -1

                        patch_W(next_patch_idx, i*2+(next_cons ?0:1)) =
                            shared_winding_number;
                        patch_W(next_patch_idx, i*2+(next_cons ?1:0)) = 
                            shared_winding_number + (next_cons ? 1:-1);
                    } else {
                        patch_W(next_patch_idx, i*2  ) = shared_winding_number;
                        patch_W(next_patch_idx, i*2+1) = shared_winding_number;
                    }
                }
                Q.push(next_patch_idx);
            }
            if (!winding_num_assigned(prev_patch_idx)) {
                const bool prev_ori = orientation[prev_i];
                const bool prev_cons = prev_ori != curr_ori;
                const size_t prev_patch_label = labels[prev_patch_idx];

                for (size_t i=0; i<num_labels; i++) {
                    const int shared_winding_number =
                        patch_W(curr_patch_idx, i*2+1);

                    if (i == prev_patch_label) {
                        // Truth table:
                        // curr_ori  next_ori  wind_# inc
                        // True      True       1
                        // True      False     -1
                        // False     True      -1
                        // False     False      1

                        patch_W(prev_patch_idx, i*2+(prev_cons ?1:0)) =
                            shared_winding_number;
                        patch_W(prev_patch_idx, i*2+(prev_cons ?0:1)) =
                            shared_winding_number + (prev_cons ? -1:1);
                    } else {
                        patch_W(prev_patch_idx, i*2  ) = shared_winding_number; 
                        patch_W(prev_patch_idx, i*2+1) = shared_winding_number; 
                    }
                }
                Q.push(prev_patch_idx);
            }
        }
    }
    assert((patch_W.array() != INVALID).all());

    using namespace propagate_winding_numbers_helper;
    return winding_number_assignment_is_consistent(orders, orientations, patch_W);
}

#if 0
template<
    typename DerivedV,
    typename DerivedF,
    typename DeriveduE,
    typename uE2EType,
    typename DerivedEMAP,
    typename DerivedC,
    typename DerivedP,
    typename DerivedW >
IGL_INLINE bool igl::cgal::propagate_winding_numbers_single_component_patch_wise(
        const Eigen::PlainObjectBase<DerivedV>& V,
        const Eigen::PlainObjectBase<DerivedF>& F,
        const Eigen::PlainObjectBase<DeriveduE>& uE,
        const std::vector<std::vector<uE2EType> >& uE2E,
        const Eigen::PlainObjectBase<DerivedEMAP>& EMAP,
        const Eigen::PlainObjectBase<DerivedC>& labels,
        const Eigen::PlainObjectBase<DerivedP>& P,
        Eigen::PlainObjectBase<DerivedW>& patch_W) {
    const size_t num_faces = F.rows();
    const size_t num_patches = P.maxCoeff() + 1;
    assert(labels.size() == num_patches);
    // Utility functions.
    auto edge_index_to_face_index = [&](size_t ei) { return ei % num_faces; };
    auto edge_index_to_corner_index = [&](size_t ei) { return ei / num_faces; };
    auto face_and_corner_index_to_edge_index = [&](size_t fi, size_t ci) {
        return ci*num_faces + fi;
    };
    auto get_edge_end_points = [&](size_t ei, size_t& s, size_t& d) {
        const size_t fi = edge_index_to_face_index(ei);
        const size_t ci = edge_index_to_corner_index(ei);
        s = F(fi, (ci+1)%3);
        d = F(fi, (ci+2)%3);
    };
    auto is_positively_orientated =
        [&](size_t fi, size_t s, size_t d) -> bool{
            const Eigen::Vector3i f = F.row(fi);
            if (f[0] == d && f[1] == s) return true;
            if (f[1] == d && f[2] == s) return true;
            if (f[2] == d && f[0] == s) return true;
            if (f[0] == s && f[1] == d) return false;
            if (f[1] == s && f[2] == d) return false;
            if (f[2] == s && f[0] == d) return false;
            throw std::runtime_error("Edge does not belong to face.");
            return false;
        };

    auto compute_signed_index = [&](size_t fi, size_t s, size_t d) -> int{
        return int(fi+1) * (is_positively_orientated(fi, s, d) ? 1:-1);
    };
    auto compute_unsigned_index = [&](int signed_idx) -> size_t {
        return abs(signed_idx) - 1;
    };

    // Order patches around each non-manifold edge.
    const size_t num_edges = uE.rows();
    std::vector<Eigen::VectorXi> orders(num_edges);
    std::vector<std::vector<bool> > orientations(num_edges);
    std::vector<std::vector<size_t> > patch_edge_adjacency(num_patches);
    for (size_t uei=0; uei<num_edges; uei++) {
        const auto& edges = uE2E[uei];
        if (edges.size() <= 2) continue;
        size_t s = uE(uei, 0);
        size_t d = uE(uei, 1);
        std::vector<int> adj_faces;
        for (size_t ei : uE2E[uei]) {
            const size_t fi = edge_index_to_face_index(ei);
            const size_t signed_fi =
                compute_signed_index(fi, s, d);
            adj_faces.push_back(signed_fi);
            patch_edge_adjacency[P[fi]].push_back(ei);
        }

        auto& order = orders[uei];
        igl::cgal::order_facets_around_edge(
                V, F, s, d, adj_faces, order);
        assert(order.minCoeff() == 0);
        assert(order.maxCoeff() == adj_faces.size() - 1);

        auto& orientation = orientations[uei];
        orientation.resize(order.size());
        std::transform(order.data(), order.data()+order.size(),
                orientation.begin(), 
                [&](size_t index) { return adj_faces[index] > 0; });
        std::transform(order.data(), order.data()+order.size(),
                order.data(), [&](size_t index) {
                return compute_unsigned_index(adj_faces[index]);
                } );
    }

    // Propagate winding number from infinity.
    // Assuming infinity has winding number 0.
    const size_t num_labels = labels.maxCoeff() + 1;
    const int INVALID = std::numeric_limits<int>::max();
    patch_W.resize(num_patches, 2*num_labels);
    patch_W.setConstant(INVALID);

    size_t outer_facet_idx;
    bool outer_facet_is_flipped;
    Eigen::VectorXi face_indices(num_faces);
    face_indices.setLinSpaced(num_faces, 0, num_faces-1);
    igl::cgal::outer_facet(V, F, face_indices,
            outer_facet_idx, outer_facet_is_flipped);
    size_t outer_patch_idx = P[outer_facet_idx];
    size_t outer_patch_label = labels[outer_patch_idx];
    patch_W.row(outer_patch_idx).setZero();
    if (outer_facet_is_flipped) {
        patch_W(outer_patch_idx, outer_patch_label*2) = -1;
    } else {
        patch_W(outer_patch_idx, outer_patch_label*2+1) = 1;
    }

    auto winding_num_assigned = [&](size_t patch_idx) -> bool{
        return (patch_W.row(patch_idx).array() != INVALID).all();
    };

    std::queue<size_t> Q;
    Q.push(outer_patch_idx);
    while (!Q.empty()) {
        const size_t curr_patch_idx = Q.front();
        const size_t curr_patch_label = labels[curr_patch_idx];
        Q.pop();

        const auto& adj_edges = patch_edge_adjacency[curr_patch_idx];
        for (size_t ei: adj_edges) {
            const size_t uei = EMAP[ei];
            const size_t fid = edge_index_to_face_index(ei);
            const auto& order = orders[uei];
            const auto& orientation = orientations[uei];
            const size_t num_adj_patches = order.size();
            assert(num_adj_patches == orientation.size());

            size_t curr_i = std::numeric_limits<size_t>::max();
            for (size_t i=0; i<num_adj_patches; i++) {
                std::cout << (orientation[i]?"+":"-") << order[i] << " ";
            }
            std::cout << std::endl;
            for (size_t i=0; i<num_adj_patches; i++) {
                if (order[i] == fid) {
                    curr_i = i;
                    break;
                }
            }
            assert(curr_i < num_adj_patches);
            const bool curr_ori = orientation[curr_i];

            const size_t next_i = curr_ori ? (curr_i+1) % num_adj_patches
                : (curr_i+num_adj_patches-1)%num_adj_patches;
            const size_t prev_i = curr_ori ?
                (curr_i+num_adj_patches-1)%num_adj_patches
                : (curr_i+1) % num_adj_patches;
            const size_t next_patch_idx = P[order[next_i]];
            const size_t prev_patch_idx = P[order[prev_i]];

            if (!winding_num_assigned(next_patch_idx)) {
                const bool next_ori = orientation[next_i];
                const bool next_cons = next_ori != curr_ori;
                const size_t next_patch_label = labels[next_patch_idx];
                for (size_t i=0; i<num_labels; i++) {
                    const int shared_winding_number =
                        patch_W(curr_patch_idx, i*2);

                    if (i == next_patch_label) {
                        // Truth table:
                        // curr_ori  next_ori  wind_# inc
                        // True      True      -1
                        // True      False      1
                        // False     True       1
                        // False     False     -1

                        patch_W(next_patch_idx, i*2+(next_cons ?0:1)) =
                            shared_winding_number;
                        patch_W(next_patch_idx, i*2+(next_cons ?1:0)) = 
                            shared_winding_number + (next_cons ? 1:-1);
                    } else {
                        patch_W(next_patch_idx, i*2  ) = shared_winding_number;
                        patch_W(next_patch_idx, i*2+1) = shared_winding_number;
                    }
                }
                Q.push(next_patch_idx);
            } else {
                const bool next_ori = orientation[next_i];
                const bool next_cons = next_ori != curr_ori;
                const size_t next_patch_label = labels[next_patch_idx];
                for (size_t i=0; i<num_labels; i++) {
                    const int shared_winding_number =
                        patch_W(curr_patch_idx, i*2);

                    if (i == next_patch_label) {
                        // Truth table:
                        // curr_ori  next_ori  wind_# inc
                        // True      True      -1
                        // True      False      1
                        // False     True       1
                        // False     False     -1

                        assert(patch_W(next_patch_idx, i*2+(next_cons ?0:1)) ==
                            shared_winding_number);
                        assert(patch_W(next_patch_idx, i*2+(next_cons ?1:0)) == 
                            shared_winding_number + (next_cons ? 1:-1));
                    } else {
                        assert(patch_W(next_patch_idx, i*2) ==
                                shared_winding_number);
                        assert(patch_W(next_patch_idx, i*2+1) ==
                                shared_winding_number);
                    }
                }
            }
            if (!winding_num_assigned(prev_patch_idx)) {
                const bool prev_ori = orientation[prev_i];
                const bool prev_cons = prev_ori != curr_ori;
                const size_t prev_patch_label = labels[prev_patch_idx];

                for (size_t i=0; i<num_labels; i++) {
                    const int shared_winding_number =
                        patch_W(curr_patch_idx, i*2+1);

                    if (i == prev_patch_label) {
                        // Truth table:
                        // curr_ori  next_ori  wind_# inc
                        // True      True       1
                        // True      False     -1
                        // False     True      -1
                        // False     False      1

                        patch_W(prev_patch_idx, i*2+(prev_cons ?1:0)) =
                            shared_winding_number;
                        patch_W(prev_patch_idx, i*2+(prev_cons ?0:1)) =
                            shared_winding_number + (prev_cons ? -1:1);
                    } else {
                        patch_W(prev_patch_idx, i*2  ) = shared_winding_number; 
                        patch_W(prev_patch_idx, i*2+1) = shared_winding_number; 
                    }
                }
                Q.push(prev_patch_idx);
            } else {
                const bool prev_ori = orientation[prev_i];
                const bool prev_cons = prev_ori != curr_ori;
                const size_t prev_patch_label = labels[prev_patch_idx];

                for (size_t i=0; i<num_labels; i++) {
                    const int shared_winding_number =
                        patch_W(curr_patch_idx, i*2+1);

                    if (i == prev_patch_label) {
                        // Truth table:
                        // curr_ori  next_ori  wind_# inc
                        // True      True       1
                        // True      False     -1
                        // False     True      -1
                        // False     False      1

                        assert(patch_W(prev_patch_idx, i*2+(prev_cons ?1:0)) ==
                            shared_winding_number);
                        assert(patch_W(prev_patch_idx, i*2+(prev_cons ?0:1)) ==
                            shared_winding_number + (prev_cons ? -1:1));
                    } else {
                        assert(patch_W(prev_patch_idx, i*2  ) ==
                                shared_winding_number); 
                        assert(patch_W(prev_patch_idx, i*2+1) ==
                                shared_winding_number); 
                    }
                }
            }
        }
    }
    assert((patch_W.array() != INVALID).all());

    using namespace propagate_winding_numbers_helper;
    if (!winding_number_assignment_is_consistent(orders, orientations, patch_W)) {
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,
            DerivedV::IsRowMajor> Vd(V.rows(), V.cols());
        for (size_t j=0; j<V.rows(); j++) {
            for (size_t k=0; k<V.cols(); k++) {
                igl::cgal::assign_scalar(V(j,k), Vd(j,k));
            }
        }
        //for (size_t j=0; j<num_faces; j++) {
        //    std::cout << patch_W(P[j], 1) << std::endl;
        //}
        igl::writeOBJ("debug_wn.obj", Vd, F);
        assert(false);
    }
    return winding_number_assignment_is_consistent(orders, orientations, patch_W);
}
#endif

template<
typename DerivedV,
typename DerivedF,
typename DerivedC,
typename DerivedW>
IGL_INLINE bool igl::cgal::propagate_winding_numbers_single_component(
        const Eigen::PlainObjectBase<DerivedV>& V,
        const Eigen::PlainObjectBase<DerivedF>& F,
        const Eigen::PlainObjectBase<DerivedC>& labels,
        Eigen::PlainObjectBase<DerivedW>& W) {
    typedef typename DerivedF::Scalar Index;
    const size_t num_faces = F.rows();

    // Extract unique edges.
    std::vector<std::vector<size_t> > uE2E;
    Eigen::Matrix<Index, Eigen::Dynamic, Eigen::Dynamic> E, uE;
    Eigen::Matrix<Index, Eigen::Dynamic, 1> EMAP;
    igl::unique_edge_map(F, E, uE, EMAP, uE2E);

    // Extract manifold patches and intersection curves.
    Eigen::VectorXi P;
    std::vector<std::vector<size_t> > intersection_curves;
    size_t num_patches =
        igl::extract_manifold_patches(F, EMAP, uE2E, P);
    igl::extract_non_manifold_edge_curves(F, EMAP, uE2E,
            intersection_curves);
    assert(P.size() == num_faces);
    assert(P.maxCoeff() + 1 == num_patches);

    Eigen::VectorXi patch_labels(num_patches);
    const int INVALID = std::numeric_limits<int>::max();
    patch_labels.setConstant(INVALID);
    for (size_t i=0; i<num_faces; i++) {
        if (patch_labels[P[i]] == INVALID) {
            patch_labels[P[i]] = labels[i];
        } else {
            assert(patch_labels[P[i]] == labels[i]);
        }
    }
    assert((patch_labels.array() != INVALID).all());
    const size_t num_labels = patch_labels.maxCoeff()+1;

    Eigen::MatrixXi winding_numbers;
    bool is_consistent =
        igl::cgal::propagate_winding_numbers_single_component_patch_wise(
            V, F, uE, uE2E, patch_labels, P, intersection_curves, winding_numbers);
    assert(winding_numbers.rows() == num_patches);
    assert(winding_numbers.cols() == 2 * num_labels);

    W.resize(num_faces, 2*num_labels);
    W.setConstant(INVALID);
    for (size_t i=0; i<num_faces; i++) {
        W.row(i) = winding_numbers.row(P[i]);
    }
    assert((W.array() != INVALID).any());

    return is_consistent;
}

template<
typename DerivedV,
typename DerivedF,
typename DerivedW>
IGL_INLINE bool igl::cgal::propagate_winding_numbers_single_component(
        const Eigen::PlainObjectBase<DerivedV>& V,
        const Eigen::PlainObjectBase<DerivedF>& F,
        Eigen::PlainObjectBase<DerivedW>& W) {
    const size_t num_faces = F.rows();
    Eigen::VectorXi labels(num_faces);
    labels.setZero();
    return igl::cgal::propagate_winding_numbers_single_component(V, F, labels, W);
}

template<
typename DerivedV,
typename DerivedF,
typename DerivedL,
typename DerivedW>
IGL_INLINE void igl::cgal::propagate_winding_numbers(
        const Eigen::PlainObjectBase<DerivedV>& V,
        const Eigen::PlainObjectBase<DerivedF>& F,
        const Eigen::PlainObjectBase<DerivedL>& labels,
        Eigen::PlainObjectBase<DerivedW>& W) {
    typedef typename DerivedF::Scalar Index;
    const size_t num_faces = F.rows();

    // Extract unique edges.
    std::vector<std::vector<size_t> > uE2E;
    Eigen::Matrix<Index, Eigen::Dynamic, Eigen::Dynamic> E, uE;
    Eigen::Matrix<Index, Eigen::Dynamic, 1> EMAP;
    igl::unique_edge_map(F, E, uE, EMAP, uE2E);

    // Check to make sure there is no boundaries and no non-manifold edges with
    // odd number of adjacent faces.
    for (const auto& adj_faces : uE2E) {
        if (adj_faces.size() % 2 == 1) {
            std::stringstream err_msg;
            err_msg << "Input mesh contains odd number of faces "
                << "sharing a single edge" << std::endl;
            err_msg << "Indicating the input mesh does not represent a valid volume"
                << ", and winding number cannot be propagated." << std::endl;
            throw std::runtime_error(err_msg.str());
        }
    }

    // Gather connected components.
    std::vector<std::vector<std::vector<Index > > > TT,_1;
    triangle_triangle_adjacency(E,EMAP,uE2E,false,TT,_1);
    Eigen::VectorXi counts;
    Eigen::VectorXi C;
    igl::facet_components(TT,C,counts);

    const size_t num_components = counts.size();
    std::vector<std::vector<size_t> > components(num_components);
    for (size_t i=0; i<num_faces; i++) {
        components[C[i]].push_back(i);
    }
    std::vector<Eigen::MatrixXi> comp_faces(num_components);
    std::vector<Eigen::VectorXi> comp_labels(num_components);
    for (size_t i=0; i<num_components; i++) {
        const auto& comp = components[i];
        auto& faces = comp_faces[i];
        auto& c_labels = comp_labels[i];
        const size_t comp_size = comp.size();
        faces.resize(comp_size, 3);
        c_labels.resize(comp_size);
        for (size_t j=0; j<comp_size; j++) {
            faces.row(j) = F.row(comp[j]);
            c_labels[j] = labels[comp[j]];
        }
    }

    // Compute winding number for each component.
    const size_t num_labels = labels.maxCoeff()+1;
    W.resize(num_faces, 2*num_labels);
    W.setZero();
    std::vector<Eigen::MatrixXi> Ws(num_components);
    for (size_t i=0; i<num_components; i++) {
        bool is_consistent =
            propagate_winding_numbers_single_component(V, comp_faces[i], comp_labels[i], Ws[i]);
        const size_t num_labels_in_i = comp_labels[i].maxCoeff()+1;
        const size_t num_faces_in_comp = comp_faces[i].rows();
        assert(Ws[i].cols() == num_labels_in_i*2);
        assert(Ws[i].rows() == num_faces_in_comp);
        const auto& comp = components[i];
        for (size_t j=0; j<num_faces_in_comp; j++) {
            const size_t fid = comp[j];
            W.block(fid, 0, 1, num_labels_in_i*2) = Ws[i].row(j);
        }

        if (!is_consistent) {
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,
                DerivedV::IsRowMajor> Vd(V.rows(), V.cols());
            for (size_t j=0; j<V.rows(); j++) {
                for (size_t k=0; k<V.cols(); k++) {
                    igl::cgal::assign_scalar(V(j,k), Vd(j,k));
                }
            }
            igl::writeOBJ("debug_wn.obj", Vd, comp_faces[i]);
            std::cout << Ws[i].col(1) << std::endl;
            std::stringstream err_msg;
            err_msg << "Component " << i
                << " has inconsistant winding number assignment." << std::endl;
            //throw std::runtime_error(err_msg.str());
            std::cout << err_msg.str() << std::endl;
        }
    }

    auto sample_component = [&](size_t cid) {
        const auto& f = comp_faces[cid].row(0).eval();
        return ((V.row(f[0]) + V.row(f[1]) + V.row(f[2])) / 3.0).eval();
    };

    std::vector<Eigen::MatrixXi> nested_Ws = Ws;
    Eigen::MatrixXi ambient_correction(num_components, 2*num_labels);
    ambient_correction.setZero();
    if (num_components > 1) {
        for (size_t i=0; i<num_components; i++) {
            DerivedV samples(num_components-1, 3);
            Eigen::VectorXi is_inside;
            auto index_without_i = [&](size_t index) {
                return index < i ? index:index-1;
            };
            for (size_t j=0; j<num_components; j++) {
                if (i == j) continue;
                samples.row(index_without_i(j)) = sample_component(j);
            }
            Eigen::VectorXi fids;
            Eigen::Matrix<bool, Eigen::Dynamic, 1> orientation;
            igl::cgal::closest_facet(V, comp_faces[i], samples,
                    fids, orientation);

            const auto& comp = components[i];
            for (size_t j=0; j<num_components; j++) {
                if (i == j) continue;
                const size_t index = index_without_i(j);
                const size_t fid = fids(index, 0);
                const bool ori = orientation(index, 0);
                for (size_t k=0; k<num_labels; k++) {
                    const int correction = W(comp[fid], k*2+(ori?0:1));
                    ambient_correction(j, k*2  ) += correction;
                    ambient_correction(j, k*2+1) += correction;
                }
            }
        }
    }

    for (size_t i=0; i<num_components; i++) {
        const auto& comp = components[i];
        const auto& correction = ambient_correction.row(i).eval();
        for (const auto& fid : comp) {
            W.row(fid) += correction;
        }
    }
}

template<
typename DerivedV,
typename DerivedF,
typename DerivedL,
typename DerivedW>
IGL_INLINE void igl::cgal::propagate_winding_numbers_beta(
        const Eigen::PlainObjectBase<DerivedV>& V,
        const Eigen::PlainObjectBase<DerivedF>& F,
        const Eigen::PlainObjectBase<DerivedL>& labels,
        Eigen::PlainObjectBase<DerivedW>& W) {
    const size_t num_faces = F.rows();
    typedef typename DerivedF::Scalar Index;

    Eigen::MatrixXi E, uE;
    Eigen::VectorXi EMAP;
    std::vector<std::vector<size_t> > uE2E;
    igl::unique_edge_map(F, E, uE, EMAP, uE2E);
    assert(propagate_winding_numbers_helper::is_orientable(F, uE, uE2E));

    Eigen::VectorXi P;
    const size_t num_patches = igl::extract_manifold_patches(F, EMAP, uE2E, P);

    DerivedW per_patch_cells;
    const size_t num_cells =
        igl::cgal::extract_cells(V, F, P, E, uE, uE2E, EMAP, per_patch_cells);

    typedef std::tuple<size_t, bool, size_t> CellConnection;
    std::vector<std::set<CellConnection> > cell_adjacency(num_cells);
    for (size_t i=0; i<num_patches; i++) {
        const int positive_cell = per_patch_cells(i,0);
        const int negative_cell = per_patch_cells(i,1);
        cell_adjacency[positive_cell].emplace(negative_cell, false, i);
        cell_adjacency[negative_cell].emplace(positive_cell, true, i);
    }

    {
        auto save_cell = [&](const std::string& filename, size_t cell_id) {
            std::vector<size_t> faces;
            for (size_t i=0; i<num_patches; i++) {
                if ((per_patch_cells.row(i).array() == cell_id).any()) {
                    for (size_t j=0; j<num_faces; j++) {
                        if (P[j] == i) {
                            faces.push_back(j);
                        }
                    }
                }
            }
            Eigen::MatrixXi cell_faces(faces.size(), 3);
            for (size_t i=0; i<faces.size(); i++) {
                cell_faces.row(i) = F.row(faces[i]);
            }
            Eigen::MatrixXd vertices(V.rows(), 3);
            for (size_t i=0; i<V.rows(); i++) {
                assign_scalar(V(i,0), vertices(i,0));
                assign_scalar(V(i,1), vertices(i,1));
                assign_scalar(V(i,2), vertices(i,2));
            }
            writePLY(filename, vertices, cell_faces);
        };

        // Check for odd cycle.
        Eigen::VectorXi cell_labels(num_cells);
        cell_labels.setZero();
        Eigen::VectorXi parents(num_cells);
        parents.setConstant(-1);
        auto trace_parents = [&](size_t idx) {
            std::list<size_t> path;
            path.push_back(idx);
            while (parents[path.back()] != path.back()) {
                path.push_back(parents[path.back()]);
            }
            return path;
        };
        for (size_t i=0; i<num_cells; i++) {
            if (cell_labels[i] == 0) {
                cell_labels[i] = 1;
                std::queue<size_t> Q;
                Q.push(i);
                parents[i] = i;
                while (!Q.empty()) {
                    size_t curr_idx = Q.front();
                    Q.pop();
                    int curr_label = cell_labels[curr_idx];
                    for (const auto& neighbor : cell_adjacency[curr_idx]) {
                        if (cell_labels[std::get<0>(neighbor)] == 0) {
                            cell_labels[std::get<0>(neighbor)] = curr_label * -1;
                            Q.push(std::get<0>(neighbor));
                            parents[std::get<0>(neighbor)] = curr_idx;
                        } else {
                            if (cell_labels[std::get<0>(neighbor)] !=
                                    curr_label * -1) {
                                std::cerr << "Odd cell cycle detected!" << std::endl;
                                auto path = trace_parents(curr_idx);
                                path.reverse();
                                auto path2 = trace_parents(std::get<0>(neighbor));
                                path.insert(path.end(),
                                        path2.begin(), path2.end());
                                for (auto cell_id : path) {
                                    std::cout << cell_id << " ";
                                    std::stringstream filename;
                                    filename << "cell_" << cell_id << ".ply";
                                    save_cell(filename.str(), cell_id);
                                }
                                std::cout << std::endl;
                            }
                            assert(cell_labels[std::get<0>(neighbor)] == curr_label * -1);
                        }
                    }
                }
            }
        }
    }

    size_t outer_facet;
    bool flipped;
    Eigen::VectorXi I;
    I.setLinSpaced(num_faces, 0, num_faces-1);
    igl::cgal::outer_facet(V, F, I, outer_facet, flipped);

    const size_t outer_patch = P[outer_facet];
    const size_t infinity_cell = per_patch_cells(outer_patch, flipped?1:0);

    Eigen::VectorXi patch_labels(num_patches);
    const int INVALID = std::numeric_limits<int>::max();
    patch_labels.setConstant(INVALID);
    for (size_t i=0; i<num_faces; i++) {
        if (patch_labels[P[i]] == INVALID) {
            patch_labels[P[i]] = labels[i];
        } else {
            assert(patch_labels[P[i]] == labels[i]);
        }
    }
    assert((patch_labels.array() != INVALID).all());
    const size_t num_labels = patch_labels.maxCoeff()+1;

    Eigen::MatrixXi per_cell_W(num_cells, num_labels);
    per_cell_W.setConstant(INVALID);
    per_cell_W.row(infinity_cell).setZero();
    std::queue<size_t> Q;
    Q.push(infinity_cell);
    while (!Q.empty()) {
        size_t curr_cell = Q.front();
        Q.pop();
        for (const auto& neighbor : cell_adjacency[curr_cell]) {
            size_t neighbor_cell, patch_idx;
            bool direction;
            std::tie(neighbor_cell, direction, patch_idx) = neighbor;
            if ((per_cell_W.row(neighbor_cell).array() == INVALID).any()) {
                per_cell_W.row(neighbor_cell) = per_cell_W.row(curr_cell);
                for (size_t i=0; i<num_labels; i++) {
                    int inc = (patch_labels[patch_idx] == i) ?
                        (direction ? -1:1) :0;
                    per_cell_W(neighbor_cell, i) =
                        per_cell_W(curr_cell, i) + inc;
                }
            } else {
                for (size_t i=0; i<num_labels; i++) {
                    if (i == patch_labels[patch_idx]) {
                        int inc = direction ? -1:1;
                        assert(per_cell_W(neighbor_cell, i) ==
                                per_cell_W(curr_cell, i) + inc);
                    } else {
                        assert(per_cell_W(neighbor_cell, i) ==
                                per_cell_W(curr_cell, i));
                    }
                }
            }
        }
    }

    W.resize(num_faces, num_labels*2);
    for (size_t i=0; i<num_faces; i++) {
        const size_t patch = P[i];
        const size_t positive_cell = per_patch_cells(patch, 0);
        const size_t negative_cell = per_patch_cells(patch, 1);
        for (size_t j=0; j<num_labels; j++) {
            W(i,j*2  ) = per_cell_W(positive_cell, j);
            W(i,j*2+1) = per_cell_W(negative_cell, j);
        }
    }
}

