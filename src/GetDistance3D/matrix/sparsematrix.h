/*******************************************************************
 * 疎行列演算ライブラリ
 *
 * May.14, 2019
 * Kazumichi INOUE <k.inoue@oyama-ct.ac.jp>
 *
 * Note:
 * 	疎行列形式のベクトルは定義しない
 *
 * Usage:
 * 	SparseMatrix A; 		// 空の疎行列の定義
 * 	SparseMatrix B(3, 3); 	// 3x3行列の定義, 要素は0
 * 	A = B.inv(); 			// AにBの逆行列を代入
 * 	SparseMatrix C = A * B;	// 疎行列の積
 * 	SparseMatrix D(3, 3);
 * 	C = D.I() * A.t() * B + D; 	// D.I()は単位行列，A.t()は転置行列
 *******************************************************************/

#ifndef __SPARSE_MATRIX_H__
#define __SPARSE_MATRIX_H__

#include "matrix.h"

struct Sparse
{
	double val;
	int row;
	int col;

	Sparse()
	{
		val = 0.0;
		row = 0;
		col = 0;
	}
};

class SparseMatrix
{
	private:
		std::vector<Sparse> m;
		int row;
		int col;

	public:
		SparseMatrix();
		SparseMatrix(int r, int c); 	// 行列のコンストラクタ

		void resize(int r, int c); 	// 行列のサイズ変更

		SparseMatrix I(); 		// 成分を単位行列化
		SparseMatrix inv(); 	// 疎行列の逆行列をSparseMatrix形式で返す

		void show(); 			// 成分のリスト表示
		void show_matrix(); 	// 成分の行列表示

		int getRows(); 		 						// 疎行列の行数を取得
		int getRows() const; 						// 疎行列の行数を取得
		int getCols(); 								// 疎行列の列数を取得
		int getCols() const;						// 疎行列の列数を取得
		int getMatrixListSize() const; 				// 疎行列リストの大きさ
		Sparse getMatrixListElment(int i) const; 	// 疎行列リストから要素番号指定でひとつ取り出す

		// 行列の要素指定してアクセスする
		const double operator()(int r, int c) const;
		double& operator()(int r, int c);

		Matrix sparse2matrix() const;

};

SparseMatrix::SparseMatrix()
{
	;
}

// 行列のコンストラクタ
SparseMatrix::SparseMatrix(int r, int c)
{
	row = r;
	col = c;
}

void SparseMatrix::resize(int r, int c)
{
	row = r;
	col = c;
}

SparseMatrix SparseMatrix::I()
{
	Sparse tmp;
	for (int i = 0;  i < row; i++) {
		tmp.val = 1.0;
		tmp.row = i;
		tmp.col = i;
		m.emplace_back(tmp);
	}

	return *this;
}

void SparseMatrix::show()
{
	if (m.size() == 0) {
		std::cout << "Non values\n";
	} else {
		std::cout << "Val\trow\tcol\n";
		for (int i = 0; i < m.size(); i++) {
			std::cout << m[i].val << "\t" << m[i].row << "\t" << m[i].col << "\n";
		}
	}
}

void SparseMatrix::show_matrix()
{
	if (m.size() == 0) {
		std::cout << "Non values\n";
	} else {
		Matrix tmp(row, col);
		for (int i = 0; i < m.size(); i++) {
			tmp(m[i].row, m[i].col) = m[i].val;
		}
		tmp.show();
	}
}

int SparseMatrix::getRows() 		// 疎行列の行数を取得
{
	return row;
}

int SparseMatrix::getRows() const	// 疎行列の行数を取得
{
	return row;
}

int SparseMatrix::getCols() 		// 疎行列の列数を取得
{
	return col;
}

int SparseMatrix::getCols() const	// 疎行列の列数を取得
{
	return col;
}

int SparseMatrix::getMatrixListSize() const  	// 疎行列リストの大きさ
{
	return m.size();
}
Sparse SparseMatrix::getMatrixListElment(int i) const  	// 疎行列リストから要素番号指定でひとつ取り出す
{
	return m[i];
}

// 行列の要素指定してアクセスする
const double SparseMatrix::operator()(int r, int c) const
{
	for (int i = 0; i < m.size(); i++) {
		if (m[i].row == r && m[i].col == c)
			return m[i].val;
	}
	return 0.0;
}

double& SparseMatrix::operator()(int r, int c)
{
	for (int i = 0; i < m.size(); i++) {
		if (m[i].row == r && m[i].col == c)
			return m[i].val;
	}
	Sparse tmp;
	tmp.row = r;
	tmp.col = c;
	m.emplace_back(tmp);
	return (m.back()).val;
}

SparseMatrix SparseMatrix::inv() // 疎行列の逆行列をSparseMatrix形式で返す
{
	if (m.size() == 0) {
		std::cout << "Non values\n";
	} else {
		Matrix tmp(row, col);
		for (int i = 0; i < m.size(); i++) {
			tmp(m[i].row, m[i].col) = m[i].val;
		}
		Matrix tmp_inv = tmp.inv();
		// Matrix -> SparseMatrix
		SparseMatrix invSparseMatrix(row, col);
		for (int i = 0; i < row; i++) {
			for (int j = 0; j < col; j++) {
				if (tmp_inv(i,j) == 0.0) {
					;
				} else {
					invSparseMatrix(i,j) = tmp_inv(i,j);
				}
			}
		}
		return invSparseMatrix;
	}
}

/********************************************
 * 四則演算のクラス外関数
 ********************************************/
SparseMatrix operator+(const SparseMatrix &A, const SparseMatrix &B)
{
	SparseMatrix C(A.getRows(), A.getCols());

	for (int i = 0; i < A.getMatrixListSize(); i++) {
		C(A.getMatrixListElment(i).row, A.getMatrixListElment(i).col)
			= A.getMatrixListElment(i).val;
	}
	for (int i = 0; i < B.getMatrixListSize(); i++) {
		C(B.getMatrixListElment(i).row, B.getMatrixListElment(i).col)
			+= B.getMatrixListElment(i).val;
	}

	return C;
}

SparseMatrix operator-(const SparseMatrix &A, const SparseMatrix &B)
{
	SparseMatrix C(A.getRows(), A.getCols());

	for (int i = 0; i < A.getMatrixListSize(); i++) {
		C(A.getMatrixListElment(i).row, A.getMatrixListElment(i).col)
			= A.getMatrixListElment(i).val;
	}
	for (int i = 0; i < B.getMatrixListSize(); i++) {
		C(B.getMatrixListElment(i).row, B.getMatrixListElment(i).col)
			-= B.getMatrixListElment(i).val;
	}

	return C;
}

SparseMatrix operator*(const SparseMatrix &A, const SparseMatrix &B)
{
	// サイズチェック
	if (A.getCols() != B.getRows()) {
		std::cout << "AとBの行・列数の不一致\n";
		exit(1);
	}

	// 疎行列用の乗算手続き
	// Aのリストの要素に対応するBの要素（無ければ0.0）を乗算する
	SparseMatrix C(A.getRows(), B.getCols());
	for (int k = 0; k < C.getCols(); k++) {
		for (int i = 0; i < A.getMatrixListSize(); i++) {
			int Arow = A.getMatrixListElment(i).row;
			int Acol = A.getMatrixListElment(i).col;
			double c = A.getMatrixListElment(i).val * B(Acol, k);
			C(Arow, k) += c;
		}
	}

	return C;
}

Matrix SparseMatrix::sparse2matrix() const
{
	Matrix tmpA(row, col);
	for (int i = 0; i < m.size(); i++) {
		tmpA(m[i].row, m[i].col) = m[i].val;
	}
	return tmpA;
}

#endif
