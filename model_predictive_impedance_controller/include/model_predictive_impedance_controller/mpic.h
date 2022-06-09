// Copyright 2022, ICube Laboratory, University of Strasbourg
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MPIC_H
#define MPIC_H

#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <fstream>

#include <qpOASES.hpp>

#include <Eigen/Dense>
#include <Eigen/QR>

USING_NAMESPACE_QPOASES

using namespace Eigen;

typedef Eigen::Matrix<double,Dynamic,Dynamic,RowMajor> RowMajMatrixXd;
/**
 * @brief The MPIC class implements the MPIC formulation and resolution
 */
class MPIC
{
public:
    /**
     * @brief MPIC - class constructor
     * @param nx - number of system states
     * @param nu - number of system inputs
     * @param N - control horizon
     */
    MPIC(int nx, int nu,int N);
    /**
     * @brief computeQP - compute QP matrices from specified problem
     */
    ~MPIC();
    void computeQP();
    /**
     * @brief set system parameters
     * @param A - state space matrix A
     * @param B - state space matrix B
     * @param K - state feedback matrix
     */
    void setSystem(MatrixXd A,MatrixXd B,MatrixXd K);
    /**
     * @brief setHorizon - set control horizon
     * @param N
     */
    void setHorizon(int N);
    /**
     * @brief setTimeStep - set sampling periode of the system
     * @param Ts
     */
    void setTimeStep(double Ts);
    /**
     * @brief addConstraintsX - add constraints on state variables
     * @param selectX - select states that should be constrained
     * @param Xmax - vector of max state values
     * @param Xmin - vector of min state values
     */
    void addConstraintsX(MatrixXd selectX, MatrixXd Xmax,MatrixXd Xmin);
    /**
     * @brief addConstraintsU - add constraints on input variables
     * @param selectu - select inputs that should be constrained
     * @param umax - vector of max input values
     * @param umin - vector of min input values
     */
    void addConstraintsU(MatrixXd selectu, MatrixXd umax, MatrixXd umin);
    /**
     * @brief firstSolveMPIC - initialize and solve first MPIC step
     * @param X0 - current state
     * @param r0 - current reference
     */
    void firstSolveMPIC(MatrixXd X0,MatrixXd r0);
    /**
     * @brief updateSolveMPIC - hotstart MPIC solution after calling firstSolveMPIC methode
     * @param X - current state
     * @param rk - current reference
     */
    void updateSolveMPIC(MatrixXd X,MatrixXd rk);
    /**
     * @brief updateK - update state feedback gain matrix
     * @param K - state feedback matrix
     */
    void updateK(MatrixXd K);
    /**
     * @brief getuOpt
     * @return optimal input
     */
    MatrixXd getuOpt();

    /**
     * @brief getHorizon
     * @return horizon of MPIC
     */
    int getHorizon();
    /**
     * @brief get Dimension ofU
     * @return dim u
     */
    int getDimU();
    /**
     * @brief get Dim X
     * @return dim x
     */
    int getDimX();
    /**
     * @brief getvalue of the Slack variable
     * @param j - j_th slack
     * @return slack value
     */
    double getSlack(int j);
    /**
     * @brief get system state according to model
     * @return state
     */
    MatrixXd getXmodel();
    /**
     * @brief QPfail flag
     */
    bool _QPfail;

private:
    int _nx,_nu,_N; // problem dimensions + control horizon
    double _Ts; // time step
    MatrixXd _K,_A,_B; // system parameters
    MatrixXd _Xmin,_Xmax,_umin,_umax; //X,u constraints
    QProblem _qpb; // MPIC problem
    RowMajMatrixXd _H, _F; // QP matrices
    MatrixXd _lbdu, _ubdu; // delta u constraints
    MatrixXd _selectX,_selectu;
    RowMajMatrixXd _Wp_sup,_Wp_inf,_Sp;
    RowMajMatrixXd _Gp;
    MatrixXd _uOpt;
    MatrixXd _duOpt;
    real_t* _qpSolution;
    MatrixXd _g_tmp,_g;
    MatrixXd _ubA,_lb,_ub;
    MatrixXd _W;
    std::vector<double> _valueSlack;
    int _nSlack;
    std::vector<double> _costSlack;
    MatrixXd _Xmodel;

};

#endif // MPIC_H
