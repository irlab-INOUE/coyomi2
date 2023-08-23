#include <iostream>
#include <vector>
#include <iomanip>
#include <cmath>
#include <opencv2/opencv.hpp>

#include "matrix.h"

int main(int argc, char* argv[])
{
	std::cout << "Hello, Matirix lib samples.\n";
	std::cout << "行列A(3, 3) を定義する(要素は0)\n";
	Matrix A(3, 3);
	A.show();
	std::cout << "\n";

	std::cout << "行列Aを単位行列にする\n";
	A.I();
	A.show();
	std::cout << "\n";

	std::cout << "A(0, 1), A(1, 0) に2を代入する\n";
	A(0, 1) = 2.0;
	A(1, 0) = 2.0;
	A.show();
	std::cout << "\n";

	std::cout << "Aの逆行列を求め，Bとする\n";
	Matrix B = A.inv();
	B.show();
	std::cout << "\n";

	std::cout << "A*Bを計算する．結果は単位行列\n";
	Matrix C = A * B;
	C.show();
	std::cout << "\n";

	std::cout << "A - A, 結果はゼロ行列\n";
	Matrix D = A - A;
	D.show();
	std::cout << "\n";

	std::cout << "A + A\n";
	Matrix E = A + A;
	E.show();
	std::cout << "\n";

	std::cout << "B * (A + A), 結果は対角成分が2の行列\n";
	Matrix F = B * (A + A);
	F.show();
	std::cout << "\n";

	std::cout << "V(3) 三次元ベクトルを定義する\n";
	Matrix V(3);
	V << 1, 1, 1;
	V.show();
	std::cout << "Vの転置(横ベクトル)\n";
	Matrix Vt = V.t();
	Vt.show();

	std::cout << "Vt * V 内積(スカラーだが，戻り値は1x1のMatrix型)\n";
	Matrix VVt = Vt * V;
	VVt.show();

	std::cout << "Gの要素をまとめて代入する\n";
	std::cout << "G << 1,2,3,4,5,6,7,8,9\n";
	Matrix G(3,3);
	G << 1,2,3,
	  4,5,6,
	  7,8,9;
	G.show();

	std::cout << "Gの対角要素にVを代入\n";
	G.diag(V, 0);
	G.show();

	std::cout << "2 * A\n";
	Matrix H = 2.0 * A;
	H.show();

	std::cout << "A * 2\n";
	Matrix I = A * 2.0;
	I.show_gorgeous();  // 表示はshow() を使っている

    std::cout << "行列x縦ベクトル\n";
    Matrix R(3,3);
    R << 1,0,0,0,1,0,0,0,1;
    Matrix T(3);
    T << 3,4,5;
    (R * T).show();

    std::cout << "横ベクトル x 行列\n";
    (T.t() * R).show();

    std::cout << "行列をランダム値で初期化\n";
    Matrix RND(3,4);
    RND.randomize();
    RND.show();

    std::cout << "行列を固有値分解\n";
    Matrix EIG(2,2);
    EIG << 0.884626, 0.301313, 0.991662, 0.933703;
    EIG.show();
    Matrix VV;
    Matrix LL;
    std::tie(VV, LL) = eigen_decomposition(EIG);
    VV.show(); std::cerr << "\n";
    LL.show(); std::cerr << "\n";

	Matrix CAST(3,3);
	CAST << 1,2,3,4,5,6,7,8,9;
	cv::Mat img = CAST.copyToCV();
	std::cout << "Bye!\n";
	return 0;
}
