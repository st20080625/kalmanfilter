#include <iostream>
#include <SDL3/SDL.h>
#include <SDL3/SDL_render.h>
#include <SDL3/SDL_ttf.h>
#include "3d.hpp"
#include "draw.hpp"
#include "kalmanfilter.hpp"
#define _USE_MATH_DEFINES
#include <cmath>
#include <vector>
#include <string>
#include <windows.h>
#include <chrono>
#include <random>

using namespace std;

void draw_car(SDL_Renderer *renderer, vec2d a, vec2d size, vec3d color)
{
	draw::Draw_Line(renderer, cpp3d::vec2d(a.x + size.x / 2, a.y), cpp3d::vec2d(a.x - size.x / 2, a.y), color);
	draw::Draw_Line(renderer, cpp3d::vec2d(a.x + size.x / 2, a.y), cpp3d::vec2d(a.x + size.x / 2, a.y - size.y), color);
	draw::Draw_Line(renderer, cpp3d::vec2d(a.x - size.x / 2, a.y), cpp3d::vec2d(a.x - size.x / 2, a.y - size.y), color);
	draw::Draw_Line(renderer, cpp3d::vec2d(a.x - size.x / 2, a.y - size.y), cpp3d::vec2d(a.x + size.x / 2, a.y - size.y), color);
}

// 内部計算はすべて弧度法で行う キー入力で0.5piずつ回転
int main()
{
	std::system("cls");
	const int window_w = 1280;
	const int window_h = 720;

	cpp3d::vec2d window_center(window_w / 2, window_h / 2);
	if (!SDL_Init(SDL_INIT_VIDEO))
	{
		return 1;
	}

	SDL_Window *window = SDL_CreateWindow("KalmanFilter Visualizer", window_w, window_h, 0);

	if (!window)
	{
		return 1;
	}

	SDL_Renderer *renderer = SDL_CreateRenderer(window, NULL);
	if (!renderer)
	{
		return 1;
	}

	SDL_RenderClear(renderer);
	SDL_SetRenderDrawColor(renderer, 100, 100, 100, 255);
	SDL_RenderPresent(renderer);

	SDL_Event event;

	// Extended Kalman Filter parameters
	// 状態
	cpp3d::matrix state_real(vector<vector<float>>{{0}, {0}, {0}}); //[x, y, theta]
	cpp3d::matrix state_estimate(vector<vector<float>>{{0}, {0}, {0}}); //[x, y, theta]
	cpp3d::matrix prev_state_estimate(vector<vector<float>>{{0}, {0}, {0}}); //[x, y, theta]
	cpp3d::matrix state_prediction(vector<vector<float>>{{0}, {0}, {0}}); //[x, y, theta]

	// 制御入力
	cpp3d::matrix control_input = cpp3d::matrix(vector<vector<float>>{{0}, {0}}); //[v, omega] v:速度, omega:角速度
	
	// 誤差共分散行列
	cpp3d::matrix covariance_predict = cpp3d::matrix(vector<vector<float>>{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}); // 初期値は単位行列
	cpp3d::matrix covariance_estimate = cpp3d::matrix(vector<vector<float>>{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}); // 初期値は単位行列

	// プロセスノイズ
	cpp3d::matrix process_noise = cpp3d::matrix(vector<vector<float>>{{0.1f, 0, 0}, {0, 0.1f, 0}, {0, 0, 0.01f}}); // 0で初期化(初期状態が完全に信じられるならば)

	// カルマンゲイン
	cpp3d::matrix kalman_gain = cpp3d::matrix(vector<vector<float>>{{0, 0}, {0, 0}, {0, 0}});

	// 観測ノイズ
	cpp3d::matrix measurement_noise = cpp3d::matrix(vector<vector<float>>{{100.0f, 0}, {0, 100.0f}}); // 0で初期化(センサの精度によって変更必須)

	// 観測間隔
	float dt = 0.1f;

	// 真値のビジュアライズ
	cpp3d::vec2d pos_real(0, 0);
	cpp3d::vec2d pos_base_x_real(1, 0);
	cpp3d::vec2d pos_base_y_real(0, 1);
	float angle_real = 0.0f;
	float angle_velocity_real = 0.0f;
	cpp3d::vec2d object_velocity(0, 0);
	// 推定値のビジュアライズ
	cpp3d::vec2d pos_estimate(0, 0);
	cpp3d::vec2d pos_base_x_estimate(1, 0);
	cpp3d::vec2d pos_base_y_estimate(0, 1);
	float angle_estimate = 0.0f;
	float angle_velocity_estimate = 0.0f;
	
	cpp3d::vec2d size(50, 50);

	auto prev_time = std::chrono::system_clock::now();
	
	std::random_device rd;
	std::mt19937 gen(rd());
	std::normal_distribution<float> noise_x(0.0f, 10.0f); // x座標のノイズ
	std::normal_distribution<float> noise_y(0.0f, 10.0f); // y座標のノイズ

	while (true)
	{
		while (SDL_PollEvent(&event))
		{
			if (event.type == SDL_EVENT_QUIT)
			{
				SDL_DestroyRenderer(renderer);
				SDL_DestroyWindow(window);
				SDL_Quit();
				return 0;
			}
		}
		auto now_time = std::chrono::system_clock::now();
		std::chrono::duration<float> elapsed = now_time - prev_time;
		dt = std::min(elapsed.count(), 0.016f); // Max FrameRate
		prev_time = now_time;
		angle_velocity_real = 0.0f;
		if (GetAsyncKeyState('Q') & 0x8000)
		{
			angle_velocity_real -= 0.5f * M_PI;
		}
		if (GetAsyncKeyState('E') & 0x8000)
		{
			angle_velocity_real += 0.5f * M_PI;
		}

		SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
		SDL_RenderClear(renderer);

		draw::Draw_Line(renderer, vec2d(0, window_h / 2), vec2d(window_w, window_h / 2), vec3d(255, 255, 255));
		draw::Draw_Line(renderer, vec2d(window_w / 2, 0), vec2d(window_w / 2, window_h), vec3d(255, 255, 255));

		cpp3d::matrix rotation_matrix_real = cpp3d::matrix(vector<vector<float>>{
			{static_cast<float>(cos(angle_real)), static_cast<float>(-sin(angle_real))},
			{static_cast<float>(sin(angle_real)), static_cast<float>(cos(angle_real))}
		});
		angle_real += angle_velocity_real * dt;

		cpp3d::matrix rotated_x_matrix_real = rotation_matrix_real.mul(cpp3d::vec2d_to_matrix(pos_base_x_real));
		cpp3d::matrix rotated_y_matrix_real = rotation_matrix_real.mul(cpp3d::vec2d_to_matrix(pos_base_y_real));

		cpp3d::vec2d rotated_x_base_real = cpp3d::matrix_to_vec2d(rotated_x_matrix_real);
		cpp3d::vec2d rotated_y_base_real = cpp3d::matrix_to_vec2d(rotated_y_matrix_real);

		cpp3d::vec2d rotated_pos_base_x_real(rotated_x_matrix_real.data[0][0], rotated_x_matrix_real.data[1][0]);
		cpp3d::vec2d rotated_pos_base_y_real(rotated_y_matrix_real.data[0][0], rotated_y_matrix_real.data[1][0]);
		// draw_car(renderer, pos_display, size, vec3d(255, 0, 0));

		control_input = cpp3d::matrix(vector<vector<float>>{{0}, {0}}); // reset control_input
		object_velocity = cpp3d::vec2d(0, 0); // reset velocity(real)
		if (GetAsyncKeyState('D') & 0x8000)
		{
			object_velocity.x += 20.0f;
		}
		if (GetAsyncKeyState('A') & 0x8000)
		{
			object_velocity.x -= 20.0f;	
		}
		if (GetAsyncKeyState('W') & 0x8000)
		{
			object_velocity.y -= 20.0f;
		}
		if (GetAsyncKeyState('S') & 0x8000)
		{
			object_velocity.y += 20.0f;
		}
		control_input.data[0][0] = object_velocity.abs();

		control_input.data[1][0] = angle_velocity_real;

		// Extended Kalman Filter prediction
		// predict state
		state_prediction = kalmanfilter::state_transition(state_estimate, control_input, dt);
		cpp3d::matrix jacobian_f = kalmanfilter::calc_jacobian_f(state_estimate, control_input, dt);
		covariance_predict = kalmanfilter::calc_error_covariance_predict(covariance_estimate, jacobian_f, process_noise);

		// update state estimate
		cpp3d::matrix predicted_measurement = kalmanfilter::observation(state_prediction);
		// create measurement with noise
		float noise_x_value = noise_x(gen);
		float noise_y_value = noise_y(gen);
		cpp3d::matrix measurement = cpp3d::matrix(vector<vector<float>>{{pos_real.x + noise_x_value}, {pos_real.y + noise_y_value}}); // real measurement + noize
		jacobian_f = kalmanfilter::calc_jacobian_h();
		kalman_gain = kalmanfilter::calc_kalman_gain(covariance_predict, jacobian_f, measurement_noise);
		state_estimate = kalmanfilter::update_state_estimate(state_prediction, kalman_gain, measurement, predicted_measurement);
		covariance_estimate = kalmanfilter::update_error_covariance(covariance_predict, kalman_gain, jacobian_f);
		// update prev_state_estimate
		prev_state_estimate = state_estimate;
		// update pos_estimate
		pos_estimate.x = state_estimate.data[0][0];
		pos_estimate.y = state_estimate.data[1][0];
		angle_estimate = state_estimate.data[2][0];
		angle_velocity_estimate = control_input.data[1][0];
		cpp3d::matrix rotation_matrix_estimate = cpp3d::matrix(vector<vector<float>>{
			{static_cast<float>(cos(angle_estimate)), static_cast<float>(-sin(angle_estimate))},
			{static_cast<float>(sin(angle_estimate)), static_cast<float>(cos(angle_estimate))}
		});
		cpp3d::vec2d rotated_x_base_estimate = cpp3d::matrix_to_vec2d(rotation_matrix_estimate.mul(cpp3d::vec2d_to_matrix(pos_base_x_real)));
		cpp3d::vec2d rotated_y_base_estimate = cpp3d::matrix_to_vec2d(rotation_matrix_estimate.mul(cpp3d::vec2d_to_matrix(pos_base_y_real)));

		// rotate_pos_real
		pos_real = pos_real + rotated_x_base_real * object_velocity.x * dt;
		pos_real = pos_real + rotated_y_base_real * object_velocity.y * dt;

		angle_real += control_input.data[1][0] * dt;

		cpp3d::vec2d pos_real_display = pos_real + window_center;
		cpp3d::vec2d pos_estimate_display = pos_estimate + window_center;
		draw::Draw_Line(renderer, pos_real_display, pos_real_display + rotated_x_base_real.scalar(size.x / 2), cpp3d::vec3d(255, 0, 0));
		draw::Draw_Line(renderer, pos_real_display, pos_real_display - rotated_y_base_real.scalar(size.y / 2), cpp3d::vec3d(0, 255, 0));
		draw::Draw_Circle(renderer, pos_real_display, 10, cpp3d::vec3d(255, 0, 0));

		draw::Draw_Line(renderer, pos_estimate_display, pos_estimate_display + rotated_x_base_estimate.scalar(size.x / 2), cpp3d::vec3d(255, 0, 0));
		draw::Draw_Line(renderer, pos_estimate_display, pos_estimate_display - rotated_y_base_estimate.scalar(size.y / 2), cpp3d::vec3d(0, 255, 0));
		draw::Draw_Circle(renderer, pos_estimate_display, 10, cpp3d::vec3d(0, 255, 255));

		// measurement visualization
		cpp3d::vec2d pos_observed(measurement.data[0][0], measurement.data[1][0]);

		draw::Draw_Circle(renderer, pos_observed + window_center, 10, cpp3d::vec3d(255, 141, 161));

		cpp3d::vec2d obs_base_x(cosf(angle_real), sinf(angle_real));
		cpp3d::vec2d obs_base_y(-sinf(angle_real), cosf(angle_real));

		draw::Draw_Line(renderer, pos_observed + window_center, 
						pos_observed + window_center + obs_base_x.scalar(size.x / 2), 
						cpp3d::vec3d(0, 0, 255));
		draw::Draw_Line(renderer, pos_observed + window_center, 
						pos_observed + window_center + obs_base_y.scalar(size.y / 2), 
						cpp3d::vec3d(0, 128, 255));

		std::cout << "\033[7A";
		// Real values
		std::cout << "Real Position: " << pos_real.x << "," << pos_real.y << "\033[K\n" << std::flush;
        std::cout << "Real Velocity: " << control_input.data[0][0] << "\033[K\n" << std::flush;
        std::cout << "Real Theta: " << angle_real * M_PI / 180.0f << "," << "Real Angle Velocity: " << angle_velocity_real << "\033[K\n" << std::flush;
		// Extended Kalman Filter estimate values
		std::cout << "Estimate Position: " << pos_estimate.x << "," << pos_estimate.y << "\033[K\n" << std::flush;
		std::cout << "Estimate Velocity: " << control_input.data[0][0] << "\033[K\n" << std::flush;
		std::cout << "Estimate Theta: " << angle_estimate * M_PI / 180.0f << "," << "Estimate Angle Velocity" << angle_velocity_estimate <<"\033[K\n" << std::flush;
		// Measurement values
		std::cout << "Measurement Position: " << pos_observed.x << "," << pos_observed.y << "\033[K\n" << std::flush;

		SDL_RenderPresent(renderer);
		SDL_Delay(10);
	}

	return 0;
}
