#include "control/BalanceCtrl.h"
#include "common/mathTools.h"
#include "common/timeMarker.h"

BalanceCtrl::BalanceCtrl(double mass, Mat3 Ib, Mat6 S, double alpha, double beta)
            : _mass(mass), _Ib(Ib), _S(S), _alpha(alpha), _beta(beta){
    _Fprev.setZero();
    _g << 0, 0, -9.81;
    _fricRatio = 0.3;
    _fricMat <<  1,  0, _fricRatio,
                -1,  0, _fricRatio,
                 0,  1, _fricRatio,
                 0, -1, _fricRatio,
                 0,  0, 1;
}

// Default setting: A1
BalanceCtrl::BalanceCtrl(QuadrupedRobot *robModel){
    Vec6 s;
    Vec12 w, u;

    _mass = robModel->getRobMass();
    _Ib = robModel->getRobInertial();
    _g << 0, 0, -9.81;

    s << 1, 1, 1, 60, 60, 60;
    _S = s.asDiagonal();
    w << 10, 10, 1, 10, 10, 1, 10, 10, 1, 10, 10, 1;
    _W = w.asDiagonal();
    u << 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1;
    _U = u.asDiagonal();
    _alpha = 0.01;
    _beta  = 0.01;

    _fricRatio = 0.3;
    _Fprev.setZero();
    // _fzmin = 0;
    _fricMat <<  1,  0, _fricRatio,
                -1,  0, _fricRatio,
                 0,  1, _fricRatio,
                 0, -1, _fricRatio,
                 0,  0, 1;
}

Vec34 BalanceCtrl::calF(Vec3 ddPcd, Vec3 dWbd, RotMat rotM, Vec34 feetPos2B, VecInt4 contact){
    calMatrixA(feetPos2B, contact);
    calVectorBd(ddPcd, dWbd, rotM);
    calConstraints(contact);

    _G = _A.transpose()*_S*_A + _alpha*_W + _beta*_U;
    _g0T = -_bd.transpose()*_S*_A - _beta*_Fprev.transpose()*_U;  // 第8.37

    solveQP();

    _Fprev = _F;
    return vec12ToVec34(_F);
}
//AF = b
void BalanceCtrl::calMatrixA(Vec34 feetPos2B, VecInt4 contact){
    for(int i(0); i < 4; ++i){
        _A.block(0, 3*i, 3, 3) = I3;
        _A.block(3, 3*i, 3, 3) = skew(feetPos2B.col(i));
    }
}
 //AF = b
void BalanceCtrl::calVectorBd(Vec3 ddPcd, Vec3 dWbd, RotMat rotM){
    //质量*速度
    _bd.head(3) = _mass * (ddPcd - _g);
    //惯量*角加速度
    _bd.tail(3) = (rotM * _Ib * rotM.transpose()) * dWbd;
}

void BalanceCtrl::calConstraints(VecInt4 contact){
    int contactLegNum = 0;
    for(int i(0); i<4; ++i){
        if(contact(i) == 1){
            contactLegNum += 1;
        }
    }

    _CI.resize(5*contactLegNum, 12); // matrix
    _ci0.resize(5*contactLegNum);  // vector 
    _CE.resize(3*(4 - contactLegNum), 12); // matrix 
    _ce0.resize(3*(4 - contactLegNum));  // vector 

    _CI.setZero();
    _ci0.setZero();
    _CE.setZero();
    _ce0.setZero();

    int ceID = 0;
    int ciID = 0;
    for(int i(0); i<4; ++i){
        if(contact(i) == 1){
            _CI.block(5*ciID, 3*i, 5, 3) = _fricMat;
            // _ci0(5*ciID + 4) = -_fzmin;
            ++ciID;
        }else{
            _CE.block(3*ceID, 3*i, 3, 3) = I3;
            ++ceID;
        }
    }

// std::cout << "_CI: " << _CI << std::endl;
// std::cout << "_CE: " << _CE << std::endl;
// std::cout << "_ci0: " << _ci0.transpose() << std::endl;

    // for(int i(0); i<4-contactLegNum; ++i){
    //     if(contact(i) == 1){
    //         _CE.block(3*constraintID, 3*i, 3, 3) = I3;
    //         ++constraintID;
    //     }
    // }

    // _CI.setZero();
    // _ci0.setZero();
    // _CE.setZero();
    // _ce0.setZero();
}

void BalanceCtrl::solveQP(){
    int n = _F.size();
    int m = _ce0.size();
    int p = _ci0.size();

    G.resize(n, n);
    CE.resize(n, m);
    CI.resize(n, p);
    g0.resize(n);
    ce0.resize(m);
    ci0.resize(p);
    x.resize(n);

    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            G[i][j] = _G(i, j);
        }
    }

    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < m; ++j) {
            CE[i][j] = (_CE.transpose())(i, j);
        } 
    }

    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < p; ++j) {
            CI[i][j] = (_CI.transpose())(i, j);
        }
    }

    for (int i = 0; i < n; ++i) {
        g0[i] = _g0T[i];
    }

    for (int i = 0; i < m; ++i) {
        ce0[i] = _ce0[i];
    }

    for (int i = 0; i < p; ++i) {
        ci0[i] = _ci0[i];
    }  // 把 cio ceo都打印出来看是不是为0

    // long long startT = getSystemTime();
    double value = solve_quadprog(G, g0, CE, ce0, CI, ci0, x);
// std::cout << "pure Cost: " << getSystemTime() - startT << std::endl;
// std::cout << "The optimal cost: " << value << std::endl;

    for (int i = 0; i < n; ++i) {
        _F[i] = x[i];
    }
}