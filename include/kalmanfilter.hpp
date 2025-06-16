#ifndef CPP_KALMANFILTER_HPP
#define CPP_KALMANFILTER_HPP
#include <vector>
#include <stdexcept>
#include "3d.hpp"

// Extended Kalman Filter
/*
予測ステップ
Xk(予測) = f(Xk-1(推定), Uk-1)                  f(X)は状態遷移関数
Pk(予測) = Fk-1 * Pk-1(推定) * Fk-1^T + Qk-1    Fはfのヤコビ行列(fのテイラー１次近似)
更新ステップ
Kk = Pk * Hk^T*(Hk * Pk * Hk^T + Rk)^-1    Kはカルマンゲイン, Hは観測関数のヤコビ行列(hのテイラー１次近似)
Xk(推定) = Xk(予測) + Kk(Zk - h(Xk(予測)))  hは観測関数
Pk(推定) = (I - Kk * Hk) * Pk(予測) 　　Iは単位行列
*/

namespace kalmanfilter {
    using std::vector;
    using cpp3d::matrix;

    //予測ステップ
    // 状態ベクトルxの予測関数 x = f(x, u):状態遷移関数
    matrix state_transition(const matrix &prev_state, const matrix &control_input, float dt);
    
    // 誤差共分散行列の予測
    matrix calc_error_covariance_predict(const matrix &prev_covariance_estimate, const matrix &jacobian_f, const matrix &process_noize);

    // 更新ステップ
    // 観測値を予測する関数 z = h(x):観測(予測)関数
    matrix observation(const matrix &state);

    // カルマンゲインの計算(更新)
    matrix calc_kalman_gain(const matrix &error_covariance, const matrix &jacobian_h, const matrix &measurement_noize);
    
    // 状態ベクトルの更新(推定)
    matrix update_state_estimate(const matrix &predicted_state, const matrix &kalman_gain, const matrix &measurement, const matrix &predicted_measurement);
    
    // 誤差共分散行列の更新(推定)
    matrix update_error_covariance(const matrix &predicted_covariance, const matrix &kalman_gain, const matrix &jacobian_h);

    // 状態遷移関数のヤコビ行列
    matrix calc_jacobian_f(const matrix &prev_state, const matrix &control_input, float dt);

    // 観測関数のヤコビ行列
    matrix calc_jacobian_h(); 
}

#endif