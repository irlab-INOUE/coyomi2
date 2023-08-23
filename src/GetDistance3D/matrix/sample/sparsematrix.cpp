#include <iostream>
#include <vector>
#include <iomanip>
#include <cmath>

#include "sparsematrix.h"

int main(int argc, char* argv[])
{
	std::cout << "Hello, Sparse Matirix lib samples.\n";

	std::cout << "行列A(3, 3) を定義する(要素は0)\n";
	SparseMatrix A(3, 3);
	A.show();
	std::cout << "\n";

	std::cout << "行列Aを単位行列にする\n";
	A.I();
	A.show();
	std::cout << "\n";
	std::cout << "行列Aを行列形式で表示\n";
	A.show_matrix();
	std::cout << "\n";

	std::cout << "A(0, 1), A(1, 0) に2を代入する\n";
	A(0, 1) = 2.0;
	A(1, 0) = 2.0;
	A.show();
	std::cout << "\n";
	A.show_matrix();

	std::cout << "Aの逆行列を求め，Bとする\n";
	SparseMatrix B = A.inv();
	B.show();
	std::cout << "\n";
	B.show_matrix();
	std::cout << "\n";

	std::cout << "疎行列同士の乗算．A*Bを計算する．結果は単位行列\n";
	SparseMatrix C = A * B;
	C.show();
	std::cout << "\n";
	C.show_matrix();
	std::cout << "\n";

	std::cout << "疎行列同士の和";
	std::cout << "A + A\n";
	SparseMatrix E = A + A;
	E.show();
	std::cout << "\n";
	E.show_matrix();
	std::cout << "\n";

	std::cout << "疎行列同士の差";
	std::cout << "A - A, 結果はゼロ行列\n";
	SparseMatrix D = A - A;
	D.show();
	std::cout << "\n";
	D.show_matrix();
	std::cout << "\n";

	std::cout << "B * (A + A), 結果は対角成分が2の行列\n";
	SparseMatrix F = B * (A + A);
	F.show_matrix();
	std::cout << "\n";

	std::cout << "Bye!\n";
	return 0;
}
