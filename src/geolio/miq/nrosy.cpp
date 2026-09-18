// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2014 Daniele Panozzo <daniele.panozzo@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.

#include "nrosy.h"

#include <igl/triangle_triangle_adjacency.h>
#include <igl/edge_topology.h>
#include <igl/per_face_normals.h>

#include <iostream>
#include <fstream>

#include <Eigen/Geometry>
#include <Eigen/Sparse>
#include <queue>

#include <gmm/gmm.h>
#include <CoMISo/Solver/ConstrainedSolver.hh>
#include <CoMISo/Solver/MISolver.hh>
#include <CoMISo/Solver/GMM_Tools.hh>
using namespace Eigen;

namespace
{
    // Backing store for the references held by a default-constructed NRosyField.
    // The class keeps its mesh inputs as const references (the caller's matrices must
    // outlive the object -- that is libigl's design), so its default constructor has
    // nothing real to bind them to. It used to bind temporaries -- V(MatrixXd()) and
    // friends -- which are destroyed as soon as the constructor returns, leaving all
    // nine references dangling. That was undefined behaviour, and since C++20 it is a
    // hard error (-Wdangling-field).
    // These objects have static storage duration, so the references stay valid for the
    // whole program and a default-constructed field simply stays inert (IsInited is
    // false) until someone calls Init().
    // Note that references cannot be rebound: a default-constructed NRosyField can
    // never be pointed at real data afterwards. Use the full constructor for that.
    const Eigen::MatrixXd emptyMatrixD;
    const Eigen::MatrixXi emptyMatrixI;
    const Eigen::VectorXd emptyVectorD;
}

igl::copyleft::comiso::NRosyField::NRosyField()
    // Members are initialised in declaration order (see nrosy.h): the
    // __Inited_TopologicalRelations flag, then softAlpha, then the mesh references,
    // with k preceding TT/TTi. The previous order put TT/TTi before k, which trips
    // -Wreorder-ctor. The two scalar members are set here because the default
    // constructor would otherwise leave them indeterminate while solve() ->
    // prepareSystemMatrix() reads softAlpha, and __Init_TopologicalRelation() reads
    // the __Inited_TopologicalRelations flag.
    : __Inited_TopologicalRelations(false),
      softAlpha(0.5),
      V(emptyMatrixD), F(emptyMatrixI), N(emptyMatrixD),
      EV(emptyMatrixI), FE(emptyMatrixI), EF(emptyMatrixI),
      k(emptyVectorD), TT(emptyMatrixI), TTi(emptyMatrixI)
{
    IsInited = false;
}

//igl::copyleft::comiso::NRosyField::NRosyField(const Eigen::MatrixXd& _V, const Eigen::MatrixXi& _F)
//    : NRosyField(_V, _F, __InitN(_V, _F), __InitEV(_V, _F), __InitFE(_V, _F), __InitEF(_V, _F), __InitTT(_V, _F), __InitTTi(_V, _F), __Initk(_V, _F))
//{
//}

igl::copyleft::comiso::NRosyField::NRosyField(
    const MatrixXd& _V, const MatrixXi& _F,
    const MatrixXd& _N,
    const MatrixXi& _EV, const MatrixXi& _FE, const MatrixXi& _EF,
    const MatrixXi& _TT, const MatrixXi& _TTi,
    const VectorXd& _k
) :
    __Inited_TopologicalRelations(false), V(_V), F(_F), N(_N), EV(_EV), FE(_FE), EF(_EF), k(_k), TT(_TT), TTi(_TTi)
{
    if (V.rows() == 0)
    {
        IsInited = false;
        return;
    }
    assert(V.rows() > 0);
    assert(F.rows() > 0);

    Init();
}

void igl::copyleft::comiso::NRosyField::__Init_TopologicalRelation(const MatrixXd& V, const MatrixXi& F)
{
    if (!__Inited_TopologicalRelations)
    {
        __Inited_TopologicalRelations = true;
        using namespace std;
        using namespace Eigen;
        // Generate topological relations
        cout << "Generating topological relations ...";
        igl::triangle_triangle_adjacency(F, __TT, __TTi);
        igl::edge_topology(V, F, __EV, __FE, __EF);
        igl::per_face_normals(V, F, __N);

        // Flag border edges
        isBorderEdge.resize(__EV.rows());
        for (unsigned i = 0; i < __EV.rows(); ++i)
            isBorderEdge[i] = (__EF(i, 0) == -1) || ((__EF(i, 1) == -1));
        computek(V, F, __N, __EV, __FE, __EF, isBorderEdge, __k);
        cout << "done." << endl;
    }
}
const MatrixXd& igl::copyleft::comiso::NRosyField::__InitN(const MatrixXd& V, const MatrixXi& F)
{
    __Init_TopologicalRelation(V, F);
    return __N;
}
const MatrixXi& igl::copyleft::comiso::NRosyField::__InitEV(const MatrixXd& V, const MatrixXi& F)
{
    __Init_TopologicalRelation(V, F);
    return __EV;
}
const MatrixXi& igl::copyleft::comiso::NRosyField::__InitFE(const MatrixXd& V, const MatrixXi& F)
{
    __Init_TopologicalRelation(V, F);
    return __FE;
}
const MatrixXi& igl::copyleft::comiso::NRosyField::__InitEF(const MatrixXd& V, const MatrixXi& F)
{
    __Init_TopologicalRelation(V, F);
    return __EF;
}
const MatrixXi& igl::copyleft::comiso::NRosyField::__InitTT(const MatrixXd& V, const MatrixXi& F)
{
    __Init_TopologicalRelation(V, F);
    return __TT;
}
const MatrixXi& igl::copyleft::comiso::NRosyField::__InitTTi(const MatrixXd& V, const MatrixXi& F)
{
    __Init_TopologicalRelation(V, F);
    return __TTi;
}
const VectorXd& igl::copyleft::comiso::NRosyField::__Initk(const MatrixXd& V, const MatrixXi& F)
{
    __Inited_TopologicalRelations = false;
    __Init_TopologicalRelation(V, F);
    return __k;
}

void igl::copyleft::comiso::NRosyField::Init()
{
    IsInited = true;

    // Flag border edges
    isBorderEdge.resize(EV.rows());
    for (unsigned i = 0; i < EV.rows(); ++i)
        isBorderEdge[i] = (EF(i, 0) == -1) || ((EF(i, 1) == -1));

    using namespace std;
    using namespace Eigen;

    // Generate reference frames
    TPs.clear();
    for (unsigned fid = 0; fid < F.rows(); ++fid)
    {
        // First edge
        Vector3d e1 = V.row(F(fid, 1)) - V.row(F(fid, 0));
        e1.normalize();
        Vector3d e2 = N.row(fid);
        e2 = e2.cross(e1);
        e2.normalize();

        MatrixXd TP(2, 3);
        TP << e1.transpose(), e2.transpose();
        TPs.push_back(TP);
    }

    // Alloc internal variables
    angles = VectorXd::Zero(F.rows());
    p = VectorXi::Zero(EV.rows());
    pFixed.resize(EV.rows());
    singularityIndex = VectorXd::Zero(V.rows());

    // Reset the constraints
    resetConstraints();

    softAlpha = 0.5;
}



void igl::copyleft::comiso::NRosyField::setSoftAlpha(double alpha)
{
    assert(alpha >= 0 && alpha <= 1);
    softAlpha = alpha;
}


void igl::copyleft::comiso::NRosyField::prepareSystemMatrix(const int N)
{
    using namespace std;
    using namespace Eigen;

    double Nd = N;

    // Minimize the MIQ energy
    // Energy on edge ij is
    //     (t_i - t_j + kij + pij*(2*pi/N))^2
    // Partial derivatives:
    //   t_i: 2     ( t_i - t_j + kij + pij*(2*pi/N)) = 0
    //   t_j: 2     (-t_i + t_j - kij - pij*(2*pi/N)) = 0
    //   pij: 4pi/N ( t_i - t_j + kij + pij*(2*pi/N)) = 0
    //
    //          t_i      t_j         pij       kij
    // t_i [     2       -2           4pi/N      2    ]
    // t_j [    -2        2          -4pi/N     -2    ]
    // pij [   4pi/N   -4pi/N    2*(2pi/N)^2   4pi/N  ]

    // Count and tag the variables
    tag_t = VectorXi::Constant(F.rows(), -1);
    vector<int> id_t;
    int count = 0;
    for (unsigned i = 0; i < F.rows(); ++i)
        if (!isHard[i])
        {
            tag_t(i) = count++;
            id_t.push_back(i);
        }

    unsigned count_t = id_t.size();

    tag_p = VectorXi::Constant(EF.rows(), -1);
    vector<int> id_p;
    for (unsigned i = 0; i < EF.rows(); ++i)
    {
        if (!pFixed[i])
        {
            // if it is not fixed then it is a variable
            tag_p(i) = count++;
        }

        // if it is not a border edge,
        if (!isBorderEdge[i])
        {
            // and it is not between two fixed faces
            if (!(isHard[EF(i, 0)] && isHard[EF(i, 1)]))
            {
                // then it participates in the energy!
                id_p.push_back(i);
            }
        }
    }

    unsigned count_p = count - count_t;
    // System sizes: A (count_t + count_p) x (count_t + count_p)
    //               b (count_t + count_p)

    b = VectorXd::Zero(count_t + count_p);

    std::vector<Triplet<double> > T;
    T.reserve(3 * 4 * count_p);

    for (unsigned r = 0; r < id_p.size(); ++r)
    {
        int eid = id_p[r];
        int i = EF(eid, 0);
        int j = EF(eid, 1);
        bool isFixed_i = isHard[i];
        bool isFixed_j = isHard[j];
        bool isFixed_p = pFixed[eid];
        int row;
        // (i)-th row: t_i [     2       -2           4pi/N      2    ]
        if (!isFixed_i)
        {
            row = tag_t[i];
            if (isFixed_i) b(row) += -2 * hard[i]; else T.push_back(Triplet<double>(row, tag_t[i], 2));
            if (isFixed_j) b(row) += 2 * hard[j]; else T.push_back(Triplet<double>(row, tag_t[j], -2));
            if (isFixed_p) b(row) += -((4 * M_PI) / Nd) * p[eid]; else T.push_back(Triplet<double>(row, tag_p[eid], ((4 * M_PI) / Nd)));
            b(row) += -2 * k[eid];
            assert(hard[i] == hard[i]);
            assert(hard[j] == hard[j]);
            assert(p[eid] == p[eid]);
            assert(k[eid] == k[eid]);
            assert(b(row) == b(row));
        }
        // (j)+1 -th row: t_j [    -2        2          -4pi/N     -2    ]
        if (!isFixed_j)
        {
            row = tag_t[j];
            if (isFixed_i) b(row) += 2 * hard[i]; else T.push_back(Triplet<double>(row, tag_t[i], -2));
            if (isFixed_j) b(row) += -2 * hard[j]; else T.push_back(Triplet<double>(row, tag_t[j], 2));
            if (isFixed_p) b(row) += ((4 * M_PI) / Nd) * p[eid]; else T.push_back(Triplet<double>(row, tag_p[eid], -((4 * M_PI) / Nd)));
            b(row) += 2 * k[eid];
            assert(k[eid] == k[eid]);
            assert(b(row) == b(row));
        }
        // (r*3)+2 -th row: pij [   4pi/N   -4pi/N    2*(2pi/N)^2   4pi/N  ]
        if (!isFixed_p)
        {
            row = tag_p[eid];
            if (isFixed_i) b(row) += -(4 * M_PI) / Nd              * hard[i]; else T.push_back(Triplet<double>(row, tag_t[i], (4 * M_PI) / Nd));
            if (isFixed_j) b(row) += (4 * M_PI) / Nd              * hard[j]; else T.push_back(Triplet<double>(row, tag_t[j], -(4 * M_PI) / Nd));
            if (isFixed_p) b(row) += -(2 * pow(((2 * M_PI) / Nd), 2)) * p[eid];  else T.push_back(Triplet<double>(row, tag_p[eid], (2 * pow(((2 * M_PI) / Nd), 2))));
            b(row) += -(4 * M_PI) / Nd * k[eid];
            assert(k[eid] == k[eid]);
            assert(b(row) == b(row));
        }

    }

    A = SparseMatrix<double>(count_t + count_p, count_t + count_p);
    A.setFromTriplets(T.begin(), T.end());

    // Soft constraints
    bool addSoft = false;

    for (unsigned i = 0; i < wSoft.size(); ++i)
        if (wSoft[i] != 0)
            addSoft = true;

    if (addSoft)
    {
        cerr << " Adding soft here: " << endl;
        cerr << " softAplha: " << softAlpha << endl;
        VectorXd bSoft = VectorXd::Zero(count_t + count_p);

        std::vector<Triplet<double> > TSoft;
        TSoft.reserve(2 * count_p);

        for (unsigned i = 0; i < F.rows(); ++i)
        {
            int varid = tag_t[i];
            if (varid != -1) // if it is a variable in the system
            {
                TSoft.push_back(Triplet<double>(varid, varid, wSoft[i]));
                bSoft[varid] += wSoft[i] * soft[i];
            }
        }
        SparseMatrix<double> ASoft(count_t + count_p, count_t + count_p);
        ASoft.setFromTriplets(TSoft.begin(), TSoft.end());

        //    ofstream s("/Users/daniele/As.txt");
        //    for(unsigned i=0; i<TSoft.size(); ++i)
        //      s << TSoft[i].row() << " " << TSoft[i].col() << " " << TSoft[i].value() << endl;
        //    s.close();

        //    ofstream s2("/Users/daniele/bs.txt");
        //    for(unsigned i=0; i<bSoft.rows(); ++i)
        //      s2 << bSoft(i) << endl;
        //    s2.close();

            // Stupid Eigen bug
        SparseMatrix<double> Atmp(count_t + count_p, count_t + count_p);
        SparseMatrix<double> Atmp2(count_t + count_p, count_t + count_p);
        SparseMatrix<double> Atmp3(count_t + count_p, count_t + count_p);

        // Merge the two part of the energy
        Atmp = (1.0 - softAlpha)*A;
        Atmp2 = softAlpha * ASoft;
        Atmp3 = Atmp + Atmp2;

        A = Atmp3;
        b = b*(1.0 - softAlpha) + bSoft * softAlpha;
    }

    //  ofstream s("/Users/daniele/A.txt");
    //  for (int k=0; k<A.outerSize(); ++k)
    //    for (SparseMatrix<double>::InnerIterator it(A,k); it; ++it)
    //    {
    //      s << it.row() << " " << it.col() << " " << it.value() << endl;
    //    }
    //  s.close();
    //
    //  ofstream s2("/Users/daniele/b.txt");
    //  for(unsigned i=0; i<b.rows(); ++i)
    //    s2 << b(i) << endl;
    //  s2.close();
}

void igl::copyleft::comiso::NRosyField::solveNoRoundings()
{
    using namespace std;
    using namespace Eigen;

    // Solve the linear system
    SimplicialLDLT<SparseMatrix<double> > solver;
    solver.compute(A);
    VectorXd x = solver.solve(b);

    // Copy the result back
    for (unsigned i = 0; i < F.rows(); ++i)
        if (tag_t[i] != -1)
            angles[i] = x(tag_t[i]);
        else
            angles[i] = hard[i];

    for (unsigned i = 0; i < EF.rows(); ++i)
        if (tag_p[i] != -1)
            p[i] = roundl(x[tag_p[i]]);
}

void igl::copyleft::comiso::NRosyField::solveRoundings(const NRosySolver solverType)
{
    using namespace std;
    using namespace Eigen;

    unsigned n = A.rows();

    gmm::col_matrix< gmm::wsvector< double > > gmm_A;
    std::vector<double> gmm_b;
    std::vector<int> ids_to_round;
    std::vector<double> x;

    gmm_A.resize(n, n);
    gmm_b.resize(n);
    x.resize(n);

    // Copy A
    for (int k = 0; k < A.outerSize(); ++k)
        for (SparseMatrix<double>::InnerIterator it(A, k); it; ++it)
        {
            gmm_A(it.row(), it.col()) += it.value();
        }

    // Copy b
    for (unsigned i = 0; i < n; ++i)
        gmm_b[i] = b[i];

    // Set variables to round
    ids_to_round.clear();
    for (unsigned i = 0; i < tag_p.size(); ++i)
        if (tag_p[i] != -1)
            ids_to_round.push_back(tag_p[i]);

    // Empty constraints
    gmm::row_matrix< gmm::wsvector< double > > gmm_C(0, n);

    COMISO::ConstrainedSolver cs;
    cs.misolver().set_gurobi_rounding(solverType == NRosySolver::gurobi_rounding);
    cs.misolver().set_cplex_rounding(solverType == NRosySolver::cplex_rounding);
    cs.misolver().set_no_rounding(solverType == NRosySolver::no_rounding);
    cs.misolver().set_direct_rounding(solverType == NRosySolver::direct_rounding);
    cs.misolver().set_multiple_rounding(solverType == NRosySolver::multiple_rounding);
    //cs.misolver().set_iterative(solverType == NRosySolver::iterative); - this will be if other options are false - so for this enumeration we dont need to set flag
    cs.solve(gmm_C, gmm_A, x, gmm_b, ids_to_round, 0.0, false, true);

    // Copy the result back
    for (unsigned i = 0; i < F.rows(); ++i)
        if (tag_t[i] != -1)
            angles[i] = x[tag_t[i]];
        else
            angles[i] = hard[i];

    for (unsigned i = 0; i < EF.rows(); ++i)
        if (tag_p[i] != -1)
            p[i] = roundl(x[tag_p[i]]);

}


void igl::copyleft::comiso::NRosyField::roundAndFix()
{
    for (unsigned i = 0; i < p.rows(); ++i)
        pFixed[i] = true;
}

void igl::copyleft::comiso::NRosyField::roundAndFixToZero()
{
    for (unsigned i = 0; i < p.rows(); ++i)
    {
        pFixed[i] = true;
        p[i] = 0;
    }
}

void igl::copyleft::comiso::NRosyField::solve(const int N, const NRosySolver solverType)
{
    std::cout << "solver for NRosyField is:   " << (int)solverType << std::endl;

    // Reduce the search space by fixing matchings
    reduceSpace();

    if (solverType == NRosySolver::SimplicialLDLT)
    {
        // Build the system
        prepareSystemMatrix(N);

        // This is a very greedy solving strategy
        // Solve with no roundings
        solveNoRoundings();

        //// Round all p and fix them
        roundAndFix();

        // Build the system
        prepareSystemMatrix(N);

        // Solve with no roundings (they are all fixed)
        solveNoRoundings();
    }
    else
    {
        // Build the system
        prepareSystemMatrix(N);

        // Solve with integer roundings
        solveRoundings(solverType);
    }
    // Find the cones
    findSingularities(N);
}

void igl::copyleft::comiso::NRosyField::setConstraintHard(const int fid, const Vector3d& v)
{
    isHard[fid] = true;
    hard(fid) = convert3DtoLocal(fid, v);
}

void igl::copyleft::comiso::NRosyField::setConstraintSoft(const int fid, const double w, const Vector3d& v)
{
    wSoft(fid) = w;
    soft(fid) = convert3DtoLocal(fid, v);
}

void igl::copyleft::comiso::NRosyField::resetConstraints()
{
    using namespace std;
    using namespace Eigen;

    isHard.resize(F.rows());
    for (unsigned i = 0; i < F.rows(); ++i)
        isHard[i] = false;
    hard = VectorXd::Zero(F.rows());

    soft = VectorXd::Zero(F.rows());
    wSoft = VectorXd::Zero(F.rows());
}

MatrixX3d igl::copyleft::comiso::NRosyField::getFieldPerFace()
{
    using namespace std;
    using namespace Eigen;

    MatrixX3d result(F.rows(), 3);
    for (unsigned i = 0; i < F.rows(); ++i)
        result.row(i) = convertLocalto3D(i, angles(i));
    return result;
}

MatrixXd igl::copyleft::comiso::NRosyField::getFFieldPerFace()
{
    using namespace std;
    using namespace Eigen;

    MatrixXd result(F.rows(), 6);
    for (unsigned i = 0; i < F.rows(); ++i)
    {
        Vector3d v1 = convertLocalto3D(i, angles(i));
        Vector3d n = N.row(i);
        Vector3d v2 = n.cross(v1);
        v1.normalize();
        v2.normalize();

        result.block(i, 0, 1, 3) = v1.transpose();
        result.block(i, 3, 1, 3) = v2.transpose();
    }
    return result;
}


void igl::copyleft::comiso::NRosyField::computek(const MatrixXd& V, const MatrixXi& F,
    const MatrixXd& N,
    const MatrixXi& EV, const MatrixXi& FE, const MatrixXi& EF,
    const std::vector<bool>& isBorderEdge,
    VectorXd& k)
{
    using namespace std;
    using namespace Eigen;

    COMISO::StopWatch sw2; sw2.start();
    cout << "computek ... (comiso) ";

    k.setZero(EV.rows());

    // For every non-border edge
    for (unsigned eid = 0; eid < EF.rows(); ++eid)
    {
        if (!isBorderEdge[eid])
        {
            int fid0 = EF(eid, 0);
            int fid1 = EF(eid, 1);

            Vector3d N0 = N.row(fid0);
            Vector3d N1 = N.row(fid1);

            // find common edge on triangle 0 and 1
            int fid0_vc = -1;
            int fid1_vc = -1;
            for (unsigned i = 0; i < 3; ++i)
            {
                if (EV(eid, 0) == F(fid0, i))
                    fid0_vc = i;
                if (EV(eid, 1) == F(fid1, i))
                    fid1_vc = i;
            }
            assert(fid0_vc != -1);
            assert(fid1_vc != -1);

            Vector3d common_edge = V.row(F(fid0, (fid0_vc + 1) % 3)) - V.row(F(fid0, fid0_vc));
            common_edge.normalize();

            // Map the two triangles in a new space where the common edge is the x axis and the N0 the z axis
            MatrixXd P(3, 3);
            VectorXd o = V.row(F(fid0, fid0_vc));
            VectorXd tmp = -N0.cross(common_edge);
            P << common_edge, tmp, N0;
            P.transposeInPlace();


            MatrixXd V0(3, 3);
            V0.row(0) = V.row(F(fid0, 0)).transpose() - o;
            V0.row(1) = V.row(F(fid0, 1)).transpose() - o;
            V0.row(2) = V.row(F(fid0, 2)).transpose() - o;

            V0 = (P*V0.transpose()).transpose();

            assert(V0(0, 2) < 10e-10);
            assert(V0(1, 2) < 10e-10);
            assert(V0(2, 2) < 10e-10);

            MatrixXd V1(3, 3);
            V1.row(0) = V.row(F(fid1, 0)).transpose() - o;
            V1.row(1) = V.row(F(fid1, 1)).transpose() - o;
            V1.row(2) = V.row(F(fid1, 2)).transpose() - o;
            V1 = (P*V1.transpose()).transpose();

            assert(V1(fid1_vc, 2) < 10e-10);
            assert(V1((fid1_vc + 1) % 3, 2) < 10e-10);

            // compute rotation R such that R * N1 = N0
            // i.e. map both triangles to the same plane
            double alpha = -atan2(V1((fid1_vc + 2) % 3, 2), V1((fid1_vc + 2) % 3, 1));

            MatrixXd R(3, 3);
            R << 1, 0, 0,
                0, cos(alpha), -sin(alpha),
                0, sin(alpha), cos(alpha);
            V1 = (R*V1.transpose()).transpose();

            assert(V1(0, 2) < 10e-10);
            assert(V1(1, 2) < 10e-10);
            assert(V1(2, 2) < 10e-10);

            // measure the angle between the reference frames
            // k_ij is the angle between the triangle on the left and the one on the right
            VectorXd ref0 = V0.row(1) - V0.row(0);
            VectorXd ref1 = V1.row(1) - V1.row(0);

            ref0.normalize();
            ref1.normalize();

            double ktemp = atan2(ref1(1), ref1(0)) - atan2(ref0(1), ref0(0));

            // just to be sure, rotate ref0 using angle ktemp...
            MatrixXd R2(2, 2);
            R2 << cos(ktemp), -sin(ktemp), sin(ktemp), cos(ktemp);

            tmp = R2*ref0.head<2>();

            // ^ is XOR in C++ and binds looser than <, so the original
            // `tmp(0) - ref1(0) < 10 ^ 10` parsed as `(tmp(0) - ref1(0) < 10) ^ 10`,
            // which is non-zero -- and therefore true in an assert -- whether or not
            // the comparison holds. The check could never fire. 1e10 is the bound the
            // expression was plainly after (10^10 as a coarse sanity limit); use
            // 10e-10, as the checks above do, to actually validate the rotation.
            assert(tmp(0) - ref1(0) < 1e10);
            assert(tmp(1) - ref1(1) < 1e10);

            k[eid] = ktemp;
            //cout << ktemp << endl;
        }
    }
    cout << "done in " << sw2.stop() / 1000.0 << " seconds" << endl;
}

void igl::copyleft::comiso::NRosyField::reduceSpace()
{
    using namespace std;
    using namespace Eigen;


    // DEBUG - Fix variables are free in the beginning
    //for (unsigned eid = 0; eid < EV.rows(); ++eid) {
    //    pFixed[eid] = true;
    //    p[eid] = 0;
    //}
    //return;


    // All variables are free in the beginning
    std::fill(pFixed.begin(), pFixed.end(), false);

    // debug
    //vector<VectorXd> debug;
    //  MatrixXd B(F.rows(),3);
  //  for(unsigned i=0; i<F.rows(); ++i)
  //    B.row(i) = 1./3. * (V.row(F(i,0)) + V.row(F(i,1)) + V.row(F(i,2)));

    vector<bool> visited(F.rows(), false);
    vector<bool> starting(F.rows(), false);

    queue<int> q;
    for (unsigned i = 0; i < F.rows(); ++i)
        if (isHard[i] || wSoft[i] != 0)
        {
            q.push(i);
            starting[i] = true;
        }

    // Reduce the search space (see MI paper)
    while (!q.empty())
    {
        int c = q.front();
        q.pop();

        visited[c] = true;
        for (int i = 0; i < 3; ++i)
        {
            int eid = FE(c, i);
            int fid = TT(c, i);

            // skip borders
            if (fid != -1)
            {
                assert((EF(eid, 0) == c && EF(eid, 1) == fid) || (EF(eid, 1) == c && EF(eid, 0) == fid));
                // for every neighbouring face
                if (!visited[fid] && !starting[fid])
                {
                    pFixed[eid] = true;
                    p[eid] = 0;
                    visited[fid] = true;
                    q.push(fid);
                    //cout << "nrosy - Skip border " << eid << " from face " << fid << endl;
                }
            }
            else
            {
                // fix borders
                pFixed[eid] = true;
                p[eid] = 0;
            }
        }
    }

    // Force matchings between hard-fixed faces
    for (unsigned i = 0; i < F.rows(); ++i)
    {
        if (isHard[i])
        {
            for (unsigned int j = 0; j < 3; ++j)
            {
                int fid = TT(i, j);
                if ((fid != -1) && (isHard[fid]))
                {
                    // i and fid are adjacent and fixed
                    int eid = FE(i, j);
                    int fid0 = EF(eid, 0);
                    int fid1 = EF(eid, 1);

                    pFixed[eid] = true;
                    p[eid] = roundl(2.0 / M_PI*(hard(fid1) - hard(fid0) - k(eid)));
                }
            }
        }
    }

    //  std::ofstream s("/Users/daniele/debug.txt");
    //  for(unsigned i=0; i<debug.size(); i += 2)
    //    s << debug[i].transpose() << " " << debug[i+1].transpose() << endl;
    //  s.close();

}

double igl::copyleft::comiso::NRosyField::convert3DtoLocal(unsigned fid, const Vector3d& v)
{
    using namespace std;
    using namespace Eigen;

    // Project onto the tangent plane
    Vector2d vp = TPs[fid] * v;

    // Convert to angle
    return atan2(vp(1), vp(0));
}

Vector3d igl::copyleft::comiso::NRosyField::convertLocalto3D(unsigned fid, double a)
{
    using namespace std;
    using namespace Eigen;

    Vector2d vp(cos(a), sin(a));
    return vp.transpose() * TPs[fid];
}

VectorXd igl::copyleft::comiso::NRosyField::angleDefect()
{
    VectorXd A = VectorXd::Constant(V.rows(), -2 * M_PI);

    for (unsigned i = 0; i < F.rows(); ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            VectorXd a = V.row(F(i, (j + 1) % 3)) - V.row(F(i, j));
            VectorXd b = V.row(F(i, (j + 2) % 3)) - V.row(F(i, j));
            double t = a.transpose()*b;
            t /= (a.norm() * b.norm());
            A(F(i, j)) += acos(t);
        }
    }

    return A;
}

void igl::copyleft::comiso::NRosyField::findSingularities(int N)
{
    // Compute I0, see http://www.graphics.rwth-aachen.de/media/papers/bommes_zimmer_2009_siggraph_011.pdf for details

    VectorXd I0 = VectorXd::Zero(V.rows());

    // first the k
    for (unsigned i = 0; i < EV.rows(); ++i)
    {
        if (!isBorderEdge[i])
        {
            I0(EV(i, 0)) -= k(i);
            I0(EV(i, 1)) += k(i);
        }
    }

    // then the A
    VectorXd A = angleDefect();

    I0 = I0 + A;

    // normalize
    I0 = I0 / (2 * M_PI);

    // round to integer (remove numerical noise)
    for (unsigned i = 0; i < I0.size(); ++i)
        I0(i) = round(I0(i));

    // compute I
    VectorXd I = I0;

    for (unsigned i = 0; i < EV.rows(); ++i)
    {
        if (!isBorderEdge[i])
        {
            I(EV(i, 0)) -= double(p(i)) / double(N);
            I(EV(i, 1)) += double(p(i)) / double(N);
        }
    }

    // Clear the vertices on the edges
    for (unsigned i = 0; i < EV.rows(); ++i)
    {
        if (isBorderEdge[i])
        {
            I0(EV(i, 0)) = 0;
            I0(EV(i, 1)) = 0;
            I(EV(i, 0)) = 0;
            I(EV(i, 1)) = 0;
            A(EV(i, 0)) = 0;
            A(EV(i, 1)) = 0;
        }
    }

    singularityIndex = I;
}

VectorXd igl::copyleft::comiso::NRosyField::getSingularityIndexPerVertex()
{
    return singularityIndex;
}

void igl::copyleft::comiso::nrosy(
    const MatrixXd& V,
    const MatrixXi& F,
    //hard constraints
    const VectorXi& b,
    const MatrixXd& bc,
    const VectorXi& b_soft,
    const VectorXd& w_soft,
    const MatrixXd& bc_soft,
    const int N,
    const double soft,
    MatrixXd& R,
    VectorXd& S
)
{

    MatrixXd _N;
    MatrixXi _EV;  MatrixXi _FE;  MatrixXi _EF;
    MatrixXi _TT;  MatrixXi _TTi;
    VectorXd _k;
    using namespace std;
    using namespace Eigen;
    // Generate topological relations
    cout << "Generating topological relations ...";
    igl::triangle_triangle_adjacency(F, _TT, _TTi);
    igl::edge_topology(V, F, _EV, _FE, _EF);
    igl::per_face_normals(V, F, _N);

    // Flag border edges
    vector<bool> isBorderEdge;
    isBorderEdge.resize(_EV.rows());
    for (unsigned i = 0; i < _EV.rows(); ++i)
        isBorderEdge[i] = (_EF(i, 0) == -1) || ((_EF(i, 1) == -1));
    NRosyField::computek(V, F, _N, _EV, _FE, _EF, isBorderEdge, _k);
    cout << "done." << endl;

    // Init solver
    igl::copyleft::comiso::NRosyField solver(V, F, _N, _EV, _FE, _EF, _TT, _TTi, _k);

    // Add hard constraints
    for (unsigned i = 0; i < b.size(); ++i)
        solver.setConstraintHard(b(i), bc.row(i));

    if (b_soft.size() > 0)
    {
        // Add soft constraints
        for (unsigned i = 0; i < b_soft.size(); ++i)
            solver.setConstraintSoft(b_soft(i), w_soft(i), bc_soft.row(i));

        // Set the soft constraints global weight
        solver.setSoftAlpha(soft);
    }


    // Interpolate
    solver.solve(N);

    // Copy the result back
    R = solver.getFieldPerFace();

    // Extract singularity indices
    S = solver.getSingularityIndexPerVertex();
}

void igl::copyleft::comiso::nrosy(
    const Eigen::MatrixXd& V,
    const Eigen::MatrixXi& F,
    const Eigen::VectorXi& b,
    const Eigen::MatrixXd& bc,
    const int N,
    Eigen::MatrixXd& R,
    Eigen::VectorXd& S
)
{
    nrosy(V, F, b, bc, VectorXi(), VectorXd(), MatrixXd(), N, 0.5, R, S);
}


int igl::copyleft::comiso::NRosyField::SizeOF()
{
    if (!IsInited)
    {
        return 0;
    }

    int r = 0;
    r += angles.size() * sizeof(double);

    // Hard constraints
    r += hard.size() * sizeof(double);
    r += isHard.size() * sizeof(bool);

    // Soft constraints
    r += soft.size() * sizeof(double);
    r += wSoft.size() * sizeof(double);


    // Mesh
    //r += V.size() * sizeof(double); - this is reference
    //r += F.size() * sizeof(double); - this is reference

    // Normals per face
    r += __N.size() * sizeof(double);

    // Per Edge information
    r += __EV.size() * sizeof(int);
    r += __FE.size() * sizeof(int);
    r += __EF.size() * sizeof(int);
    r += isBorderEdge.size() * sizeof(bool);

    // Face Topology
    r += __TT.size() * sizeof(int);
    r += __TTi.size() * sizeof(int);

    // Angle between two reference frames
    r += __k.size() * sizeof(double);

    // Jumps
    r += p.size() * sizeof(int);
    r += pFixed.size() * sizeof(bool);

    // Singularity index
    r += singularityIndex.size() * sizeof(double);

    // Reference frame per triangle
    r += TPs.size() * 2 * 3 * sizeof(double);


    // System stuff
    r += A.data().allocatedSize()*(sizeof(double) + sizeof(int));
    r += b.size() * sizeof(double);
    r += tag_t.size() * sizeof(int);
    r += tag_p.size() * sizeof(int);

    return r;
}