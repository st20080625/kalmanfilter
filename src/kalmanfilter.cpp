#define _USE_MATH_DEFINES
#include "3d.hpp"
#include "kalmanfilter.hpp"
#include <cmath>
#include <random>

// invがcpp3dで4*4行列までにしか対応していないため,拡張が必要. 今度やる
namespace kalmanfilter{
    using std::vector;
    using cpp3d::matrix;

    // 予測ステップ
    // 状態遷移関数
    matrix state_transition(const matrix &prev_state, const matrix &control_input, float dt) {
        float v = control_input.data[0][0];
        float omega = control_input.data[1][0];
        
        // プロセスノイズを生成（毎回異なる値）
        static std::random_device rd;
        static std::mt19937 gen(rd());
        static std::normal_distribution<float> noise_x(0, 0.5f);  // 標準偏差は調整可能
        static std::normal_distribution<float> noise_y(0, 0.5f);
        static std::normal_distribution<float> noise_theta(0, 0.05f);
        
        // ノイズを加えた状態遷移
        return matrix(vector<vector<float>>{
            {prev_state.data[0][0] + v * dt * cosf(prev_state.data[2][0]) + noise_x(gen)},
            {prev_state.data[1][0] + v * dt * sinf(prev_state.data[2][0]) + noise_y(gen)},
            {prev_state.data[2][0] + omega * dt + noise_theta(gen)}
        });

    }

    // 誤差共分散行列の予測
    matrix calc_error_covariance_predict(const matrix &prev_covariance_estimate, const matrix &jacobian_f, const matrix &process_noize) {
        return jacobian_f.mul(prev_covariance_estimate).mul(jacobian_f.transpose()).add(process_noize);
    }

    // 更新ステップ
    // 観測値を予測する関数 z = h(x):観測(予測)関数
    matrix observation(const matrix &state){
        // 観測関数の実装
        // ここでは単純なモデルを使用
        return matrix(vector<vector<float>>{
            {state.data[0][0]}, // x座標
            {state.data[1][0]}  // y座標
        });
    }

    // カルマンゲインの計算(更新)
    matrix calc_kalman_gain(const matrix &error_covariance, const matrix &jacobian_h, const matrix &measurement_noize) {
        return error_covariance.mul(jacobian_h.transpose()).mul(
            jacobian_h.mul(error_covariance).mul(jacobian_h.transpose()).add(measurement_noize).inverse());
    }

    // 状態ベクトルの更新(推定)
    matrix update_state_estimate(const matrix &predicted_state, const matrix &kalman_gain, const matrix &measurement, const matrix &predicted_measurement) {
        return predicted_state.add(kalman_gain.mul(measurement.sub(predicted_measurement)));
    }

    // 誤差共分散行列の更新(推定)
    matrix update_error_covariance(const matrix &predicted_covariance, const matrix &kalman_gain, const matrix &jacobian_h) {
        vector<vector<float>> identity(predicted_covariance.rows, vector<float>(predicted_covariance.rows, 0.0f));
        for (int i = 0; i < predicted_covariance.rows; ++i) {
            identity[i][i] = 1.0f;
        }
        matrix identity_matrix(identity);
        return identity_matrix.sub(kalman_gain.mul(jacobian_h)).mul(predicted_covariance);
    }

    // 状態遷移関数のヤコビ行列(テイラー１次近似)
    matrix calc_jacobian_f(const matrix &prev_state, const matrix &control_input, float dt) {
        float v = control_input.data[0][0];
        float omega = control_input.data[1][0];
        return matrix(vector<vector<float>>{
            {1, 0, -v * dt * sinf(prev_state.data[2][0])},
            {0, 1, v * dt * cosf(prev_state.data[2][0])},
            {0, 0, 1}
        });
    }

    // 観測関数のヤコビ行列(テイラー１次近似)
    matrix calc_jacobian_h() {
        return matrix(vector<vector<float>>{
            {1, 0, 0}, // x座標に対する偏微分
            {0, 1, 0}  // y座標に対する偏微分
        });
    }

}