/*******************************************************************
 * 行列演算ライブラリ
 *
 * Jan.31, 2019
 * Kazumichi INOUE <k.inoue@oyama-ct.ac.jp>
 *
 * Usage:
 * 	Matrix A;	// 空の行列の定義
 * 	Matrix B(3, 3); 	// 3x3行列の定義, 要素は0
 * 	A = B.inv(); 		// AにBの逆行列を代入
 * 	Matrix C = A * B;	// 行列の積
 * 	Matrix D(3, 3);
 * 	C = D.I() * A.t() * B + D; 	// D.I()は単位行列，A.t()は転置行列
 *
 * Special Thanks
 * 	Kazuharu Nakajima and 1SE Members
 * 	中嶋氏からは参照返し，const 修飾子について極めて重要な貢献があった
 *
 * Reference:
 * https://qiita.com/h_hiro_/items/a6484101d87847299885
 *
 * Note:
 * - Jul. 6, 2019 シフト演算子で要素の定義をできるように改変
 *******************************************************************/

#ifndef __MATRIX_H__
#define __MATRIX_H__
#include <iostream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <random>
#include <tuple>
#include <omp.h>
#include <opencv2/opencv.hpp>

class Matrix
{
	private:
		// シフト演算子のオーバーロードに必要なカンマ演算子のオーバーロード
		class Separator{
			private:
				Matrix &m_;
			public:
				Separator(Matrix &ml) : m_(ml) {}

				Separator &operator,(double value){
					m_.m.push_back(value);
					return *this;
				}
		};
		//-------------------
		std::vector<double> m; 	// 行列の要素を格納(1次元配列)
		int rows; 				// 行数
		int cols; 				// 列数

		// 行・列の入れ替え
		void pivot_r(std::vector<double> &a, int i, int j); 	// 行の入れ替え
		void pivot_c(std::vector<double> &a, int i, int j); 	// 列の入れ替え
		void pivot_c(Matrix &a, int i, int j); 					// 列の入れ替え（行列を引数にとる場合）

	public:
		Matrix();
		~Matrix();
		Matrix(const int r, const int c); 	// 行列のコンストラクタ
		Matrix(const int r); 			// ベクトルのコンストラクタ

		void resize(const int r, const int c); 	// 行列のサイズ変更
		void resize(const int r); 		// ベクトルのサイズ変更
		void substitute(std::vector<double> &tmp);// ベクトルの代入

		// シフト演算子<< をオーバーロード
		Separator operator<<(double value) {
			m.assign(1, value);
			return(Separator(*this));
		}

		Matrix I(); 				// 成分を単位行列化
		Matrix t(); 				// 転置
		Matrix inv(); 				// 逆行列(ガウス・ジョルダン法
		double det(int n);				// 行列式
		Matrix diag(Matrix &v, int k); 	// 対角要素を配置する

        Matrix randomize();         // 乱数で要素を初期化

		void show(); 				// 成分の表示
		void show_gorgeous(); 		// 括弧付き

		std::vector<double>::iterator getItrBegin(); 	// 行列要素のベクトルの最初のイテレータ
		std::vector<double>::iterator getItrEnd(); 	    // 行列要素のベクトルの最後のイテレータ

		// 行列の行列数を取得する
		int getRows();
		int getRows() const;
		int getCols();
		int getCols() const;

		// 行列の要素指定してアクセスする
		double& operator()(const int r, const int c);
		const double& operator()(const int r, const int c) const;

		// ベクトルの要素指定してアクセスする
		double& operator()(int n);
		const double& operator()(int n) const;

		// Matrix -> cv::Mat へ変換
		cv::Mat copyToCV();
};

extern Matrix operator+(const Matrix &A, const Matrix &B);
extern Matrix operator-(const Matrix &A, const Matrix &B);
extern Matrix operator*(const Matrix &A, const Matrix &B);
extern Matrix operator*(const double &k, Matrix &A);
extern Matrix operator*(Matrix &A, const double &k);
#endif
