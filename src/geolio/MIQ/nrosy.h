// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2014 Daniele Panozzo <daniele.panozzo@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.
#ifndef IGL_COMISO_NROSY_H
#define IGL_COMISO_NROSY_H

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <vector>
#include <igl/igl_inline.h>
using namespace Eigen;

;
enum  class NRosySolver
{
    NPolyVectorField,
    NPolyVectorFieldGeneral,
    FrameField,
    SimplicialLDLT,
    gurobi_rounding,  //solve_gurobi(_A, _x, _rhs, _to_round);
    cplex_rounding,    //solve_cplex(_A, _x, _rhs, _to_round);
    no_rounding,       //solve_no_rounding(_A, _x, _rhs);
    direct_rounding,    //solve_direct_rounding( _A, _x, _rhs, _to_round);
    multiple_rounding, //solve_multiple_rounding(_A, _x, _rhs, _to_round);
    iterative                 //solve_iterative(_A, _x, _rhs, _to_round, _fixed_order);
};
namespace igl
{
  namespace copyleft
  {
  namespace comiso
  {


      class NRosyField
      {
      public:
          // Init
          bool __Inited_TopologicalRelations;
          bool IsInited;
          NRosyField();
          //IGL_INLINE NRosyField(const MatrixXd& _V, const MatrixXi& _F);
          NRosyField(const MatrixXd& _V, const MatrixXi& _F,
              const MatrixXd &_N,
              const MatrixXi &_EV, const MatrixXi &_FE, const MatrixXi &_EF,
              const MatrixXi &_TT, const MatrixXi &_TTi,
              const VectorXd& _k);
          void Init();

          // Generate the N-rosy field
          // N degree of the rosy field
          // roundseparately: round the integer variables one at a time, slower but higher quality
          void solve(const int N = 4, NRosySolver solverType = NRosySolver::multiple_rounding);

          // Set a hard constraint on fid
          // fid: face id
          // v: direction to fix (in 3d)
          void setConstraintHard(const int fid, const Eigen::Vector3d& v);

          // Set a soft constraint on fid
          // fid: face id
          // w: weight of the soft constraint, clipped between 0 and 1
          // v: direction to fix (in 3d)
          void setConstraintSoft(const int fid, const double w, const Eigen::Vector3d& v);

          // Set the ratio between smoothness and soft constraints (0 -> smoothness only, 1 -> soft constr only)
          void setSoftAlpha(double alpha);

          // Reset constraints (at least one constraint must be present or solve will fail)
          void resetConstraints();

          // Return the current field
          Eigen::MatrixX3d getFieldPerFace();

          // Return the current field (in Ahish's ffield format)
          Eigen::MatrixXd getFFieldPerFace();

          // Compute singularity indexes
          void findSingularities(int N);

          // Return the singularities
          Eigen::VectorXd getSingularityIndexPerVertex();




          // Remove useless matchings
          void reduceSpace();

          // Prepare the system matrix
          void prepareSystemMatrix(const int N);

          // Solve without roundings
          void solveNoRoundings();

          // Solve with roundings using CoMIso
          void solveRoundings(NRosySolver solverType);

          // Round all p to 0 and fix
          void roundAndFixToZero();

          // Round all p and fix
          void roundAndFix();

          // Convert a vector in 3d to an angle wrt the local reference system
          double convert3DtoLocal(unsigned fid, const Eigen::Vector3d& v);

          // Convert an angle wrt the local reference system to a 3d vector
          Eigen::Vector3d convertLocalto3D(unsigned fid, double a);

          // Compute the per vertex angle defect
          Eigen::VectorXd angleDefect();


          // Temporary variable for the field
          Eigen::VectorXd angles;

          // Hard constraints
          Eigen::VectorXd hard;
          std::vector<bool> isHard;

          // Soft constraints
          Eigen::VectorXd soft;
          Eigen::VectorXd wSoft;
          double          softAlpha;




          // Jumps
          Eigen::VectorXi p;
          std::vector<bool> pFixed;

          //------
          //------
          //------
          // Mesh
          const MatrixXd& V;
          const MatrixXi& F;

          // Normals per face
          const MatrixXd& N;

          // Edge Topology
          const MatrixXi& EV;
          const MatrixXi& FE;
          const MatrixXi& EF;
          std::vector<bool> isBorderEdge;

          // Angle between two reference frames (Per Edge information)
          const VectorXd& k;

          // Face Topology
          const MatrixXi& TT;
          const MatrixXi& TTi;

          MatrixXd __N;
          MatrixXi __EV;
          MatrixXi __FE;
          MatrixXi __EF;
          MatrixXi __TT;
          MatrixXi __TTi;
          VectorXd __k;
          //------
          //------
          //------

          // Singularity index
          Eigen::VectorXd singularityIndex;

          int SizeOF();
          // Compute angle differences between reference frames
          static void computek(const MatrixXd& V, const MatrixXi& F,
              const MatrixXd& N,
              const MatrixXi& EV, const MatrixXi& FE, const MatrixXi& EF,
              const std::vector<bool>& isBorderEdge,
              VectorXd& k);
      private:

          // Reference frame per triangle
          std::vector<Eigen::MatrixXd> TPs;

          // System stuff
          Eigen::SparseMatrix<double> A;
          Eigen::VectorXd b;
          Eigen::VectorXi tag_t;
          Eigen::VectorXi tag_p;

          void __Init_TopologicalRelation(const MatrixXd& V, const MatrixXi& F);
          const MatrixXd& __InitN(const MatrixXd& V, const MatrixXi& F);
          const MatrixXi& __InitEV(const MatrixXd& V, const MatrixXi& F);
          const MatrixXi& __InitEF(const MatrixXd& V, const MatrixXi& F);
          const MatrixXi& __InitFE(const MatrixXd& V, const MatrixXi& F);
          const MatrixXi& __InitTT(const MatrixXd& V, const MatrixXi& F);
          const MatrixXi& __InitTTi(const MatrixXd& V, const MatrixXi& F);
          const VectorXd& __Initk(const MatrixXd& V, const MatrixXi& F);

      };

    // Generate a N-RoSy field from a sparse set of constraints
    //
    // Inputs:
    //   V       #V by 3 list of mesh vertex coordinates
    //   F       #F by 3 list of mesh faces (must be triangles)
    //   b       #B by 1 list of constrained face indices
    //   bc      #B by 3 list of representative vectors for the constrained faces
    //   b_soft  #S by 1 b for soft constraints
    //   w_soft  #S by 1 weight for the soft constraints (0-1)
    //   bc_soft #S by 3 bc for soft constraints
    //   N       the degree of the N-RoSy vector field
    //   soft    the strenght of the soft contraints w.r.t. smoothness
    //           (0 -> smoothness only, 1->constraints only)
    // Outputs:
    //   R       #F by 3 the representative vectors of the interpolated field
    //   S       #V by 1 the singularity index for each vertex (0 = regular)
    void nrosy(
      const Eigen::MatrixXd& V,
      const Eigen::MatrixXi& F,
      const Eigen::VectorXi& b,
      const Eigen::MatrixXd& bc,
      const Eigen::VectorXi& b_soft,
      const Eigen::VectorXd& w_soft,
      const Eigen::MatrixXd& bc_soft,
      const int N,
      const double soft,
      Eigen::MatrixXd& R,
      Eigen::VectorXd& S
      );
    //wrapper for the case without soft constraints
    void nrosy(
     const Eigen::MatrixXd& V,
     const Eigen::MatrixXi& F,
     const Eigen::VectorXi& b,
     const Eigen::MatrixXd& bc,
     const int N,
     Eigen::MatrixXd& R,
     Eigen::VectorXd& S
      );

  }
}
}

// #ifndef IGL_STATIC_LIBRARY
// #  include "nrosy.cpp"
// #endif

#endif