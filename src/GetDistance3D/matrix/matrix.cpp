#include "matrix.h"

Matrix::Matrix()
{
	;
}

Matrix::~Matrix()
{
	;
}

Matrix::Matrix(const int row, const int col)
{
	m.resize(row * col);
	rows = row;
	cols = col;
	std::vector<double>::iterator itr;
	for(itr = m.begin(); itr != m.end(); ++itr) {
		*itr = 0.0;
	}
}

Matrix::Matrix(const int row)
{
	m.resize(row);
	rows = row;
	cols = 1;
	std::vector<double>::iterator itr;
	for(itr = m.begin(); itr != m.end(); ++itr) {
		*itr = 0.0;
	}
}

void Matrix::resize(const int row, const int col)  	// 行列のサイズ変更
{
	m.resize(row * col);
	m.shrink_to_fit();

	rows = row;
	cols = col;
	std::vector<double>::iterator itr;
	for(itr = m.begin(); itr != m.end(); ++itr) {
		*itr = 0.0;
	}
}

void Matrix::resize(const int row)  				// ベクトルのサイズ変更
{
	m.resize(row);
	m.shrink_to_fit();

	rows = row;
	cols = 1;
	std::vector<double>::iterator itr;
	for(itr = m.begin(); itr != m.end(); ++itr) {
		*itr = 0.0;
	}
}

void Matrix::substitute(std::vector<double> &tmp){
	Matrix(tmp.size());
	for(int i = 0; i < tmp.size(); i++){
			m[i] = tmp[i];
	}
}

Matrix Matrix::I() 				// 成分を単位行列化
{
	// 行列の要素をクリア
	std::vector<double> clr(rows*cols, 0.0);
	m = clr;
	std::vector<double>::iterator itr;
	for (itr = m.begin(); itr < m.end();) {
		*itr = 1.0;
		itr = itr + cols + 1;
	}
	return *this;
}

Matrix Matrix::t() 				// 転置行列（自分自身は変化しない)
{
	std::vector<double>::iterator itr;
	itr = m.begin();

	Matrix tmp(cols, rows);

	for (int i = 0; i < rows; i++) {
		for (int j = 0; j < cols; j++) {
			tmp(j, i) = *itr;
			++itr;
		}
	}
	return tmp;
}

// 成分の表示
void Matrix::show()
{
	std::vector<double>::iterator itr;
	int count = 0;

	if (cols != 0 && rows != 0) { // 行列の場合
		for(itr = m.begin(); itr != m.end(); ++itr) {
			count++;
			std::cout << *itr;
			if ((count % cols) != 0) {
				std::cout << "\t";
			} else {
				std::cout << "\n";
			}
		}
	} else { // ベクトルの場合
		// 列ベクトルの場合
		if (rows > 0) {
			for(itr = m.begin(); itr != m.end(); ++itr) {
				std::cout << *itr << "\n";
			}
		} else { // 行ベクトルの場合
			for(itr = m.begin(); itr != m.end(); ++itr) {
				std::cout << *itr << "\t";
			}
			std::cout << "\n";
		}
	}
}

// 括弧付き
void Matrix::show_gorgeous()
{
    show();
}

std::vector<double>::iterator Matrix::getItrBegin()  	// 行列要素のベクトルの最初のイテレータ
{
	return m.begin();
}

std::vector<double>::iterator Matrix::getItrEnd()  	// 行列要素のベクトルの最後のイテレータ
{
	return m.end();
}

// 要素を指定する()オペレータ
//// 行列
const double& Matrix::operator()(const int r, const int c) const
{
	//if (r >= rows || c >= cols || r < 0 || c < 0) {
	//	std::cerr << "in matrix error:行列の範囲外の要素を指定しています\n";
	//	exit(0);
	//}
	return m[r * cols + c];
}

double& Matrix::operator()(const int r, const int c)
{
	//if (r >= rows || c >= cols || r < 0 || c < 0) {
	//	std::cerr << "in matrix error:行列の範囲外の要素を指定しています\n";
	//	exit(0);
	//}
	return m[r * cols + c];
}


//// ベクトル
const double& Matrix::operator()(const int n) const
{
	return m[n];
}

double& Matrix::operator()(const int n)
{
	return m[n];
}

int Matrix::getRows()
{
	return rows;
}

int Matrix::getRows() const
{
	return rows;
}

int Matrix::getCols()
{
	return cols;
}

int Matrix::getCols() const
{
	return cols;
}

void Matrix::pivot_r(std::vector<double> &a, int i, int j)
{
	// i, j行の入れ替え
	// i,j 行目の先頭のイテレータを得る
	std::vector<double>::iterator row_i_first, row_j_first;
	row_i_first = a.begin() + i * cols;
	row_j_first = a.begin() + j * cols;

	for (int k = 0; k < cols; ++k) {
		double tmp = *(row_j_first + k);

		*(row_j_first + k) = *(row_i_first + k);
		*(row_i_first + k) = tmp;
	}
}

void Matrix::pivot_c(std::vector<double> &a, int i, int j)
{
	// i, j列の入れ替え
	// i, j列目の先頭のイテレータを得る
	std::vector<double>::iterator col_i_first, col_j_first;
	col_i_first = a.begin() + i;
	col_j_first = a.begin() + j;

	for (int k = 0; k < rows; ++k) {
		double tmp = *(col_i_first + k * cols);
		*(col_i_first + k * cols) = *(col_j_first + k * cols);
		*(col_j_first + k * cols) = tmp;
	}
}

void Matrix::pivot_c(Matrix &a, int i, int j)
{
	// i, j列の入れ替え
	// i, j列目の先頭のイテレータを得る
	std::vector<double>::iterator col_i_first, col_j_first;
	col_i_first = a.getItrBegin() + i;
	col_j_first = a.getItrBegin() + j;

	for (int k = 0; k < rows; ++k) {
		double tmp = *(col_i_first + k * cols);
		*(col_i_first + k * cols) = *(col_j_first + k * cols);
		*(col_j_first + k * cols) = tmp;
	}
}

Matrix Matrix::inv()
{
	// 結果を返すための行列
	Matrix m_inv(rows, cols);

	// 作業用に元の行列をコピーする
	std::vector<double> tmp;
	tmp = m;

    // 結果の一時保存用
    Matrix result(rows, 2 * cols);

    double pivot, delta;
    int x, y, i, j, k;

    int N = rows;

    for (int y = 0; y < N; y++ ) {
        for ( x = 0; x < N; x++ ) {
            result(y, x) = tmp[y * cols + x];
            result(y, x + N) = 0.0;
        }
        result(y, y + N) = 1.0;
    }

    for (int i = 0; i < N; i++ ) {
        pivot = result(i,i);
        if ( fabs(pivot) < 0.00001 ) {
            printf("ピボット数が許容誤差以下\n");
            return 1;
        }

        for (int j = 1; j < 2*N; j++ )
            result(i, j) /= pivot;

        for (int k = 0; k < N; k++ ) {
            delta = result(k, i);

            for (int j = i; j < 2*N; j++ )
                if ( k != i )
                    result(k, j) -= delta * result(i, j);
        }
    }

    // resultから逆行列の分を抜き出す
    for (int i = 0; i < rows; i++)
        for (int j = 0; j < cols; j++)
            m_inv(i, j) = result(i, cols + j);

	return m_inv;
}

double Matrix::det(int n) {
	if(cols < n || rows < n){
		std::cout<<"Error while solving determinant..\n";
		exit(1);
	}
   double d = 0.0;
   Matrix submatrix(rows, cols);
   if (n == 2)
      return ((m[0] * m[1+cols]) - (m[cols] * m[1]));
   else {
      for (int x = 0; x <n ; x++) {
            int subi = 0;
            for (int i = 1; i < n; i++) {
               int subj = 0;
               for (int j = 0; j < n; j++) {
                  if (j == x)
                  continue;
                  submatrix(subi, subj) = m[i*cols+j];
                  subj++;
               }
               subi++;
            }
            d += (pow(-1.0, x) * m[x] * submatrix.det(n-1));
      }
   }
   return d;
}

Matrix Matrix::diag(Matrix &v, int k)
{
	// 対角要素を配置する
	// Matrix &v 	配置する対角要素のベクトル
	// int k 		配置する場所, k=0: 対角要素, k>0: 上対角要素, k<0: 下対角要素

	std::vector<double>::iterator itr; 						// 自分自身の行列要素のイテレータ
	std::vector<double>::iterator itr_v = v.getItrBegin(); 	// ベクトルVの要素のイテレータ

	if (k == 0) {
		for (itr = m.begin(); itr < m.end(); itr += cols + 1) {
			*itr = *itr_v;
			++itr_v;
		}
	} else if (k > 0) {
		for (itr = m.begin() + k; itr < m.end(); itr += cols + 1) {
			*itr = *itr_v;
			++itr_v;
		}
	} else if (k < 0) {
		for (itr = m.begin() + abs(k)*cols; itr < m.end(); itr += cols + 1) {
			*itr = *itr_v;
			++itr_v;
		}
	}

	return *this;
}

Matrix Matrix::randomize()
{
    std::random_device rnd;     // 非決定的な乱数生成器を生成
    std::mt19937 mt(rnd());     //  メルセンヌ・ツイスタの32ビット版、引数は初期シード値
    // 乱数で要素を初期化
    for (int j = 0; j < rows; j++) {
        for (int i = 0; i < cols; i++) {
            //std::uniform_real_distribution<> rand_number(0.0, 1.0);        // [0, 1.0] 範囲の一様乱数
            std::normal_distribution<> rand_number(0.0, 1.0);
            m[j*cols + i] = rand_number(mt);
        }
    }
    return *this;
}

std::tuple<Matrix, Matrix> eigen_decomposition(Matrix &Awork)
{
    // 行列Aを固有値分解する
    Matrix A = Awork;

    std::vector<double> lambda;
    Matrix V(A.getRows(), A.getCols());
    Matrix L(A.getRows(), A.getCols());
    // 適当な単位ベクトルを用意する
    Matrix X(A.getRows(), 1);
    X.randomize();
    double sum = 0.0;
    for (int i = 0; i < X.getRows(); i++) sum += X(i, 0) * X(i, 0);
    sum = sqrt(sum);
    for (int i = 0; i < X.getRows(); i++) X(i, 0) = X(i, 0) / sum;

    for (int k = 0; k < X.getRows(); k++) {
        for (int i = 0; i < 50; i++) {
            Matrix tmp = A * X;
            X = tmp;
            double sum = 0.0;
            for (int i = 0; i < X.getRows(); i++) sum += X(i, 0) * X(i, 0);
            sum = sqrt(sum);
            for (int i = 0; i < X.getRows(); i++) X(i, 0) = X(i, 0) / sum;
            //X.show(); std::cerr << "\n";
        }
        Matrix lam = A * X;
        double absol = 0.0;
        for (int i = 0; i < lam.getRows(); i++) {
            absol += lam(i, 0) * lam(i, 0);
        }
        absol = sqrt(absol);
        lambda.emplace_back(absol);
        A = A - absol * X * X.t();

        for (int i = 0; i < X.getRows(); i++) {
            V(i, k) = X(i, 0);
        }
    }

    for (int i = 0; i < lambda.size(); i++) {
        L(i,i) = lambda[i];
    }

    return {V, L};
}

cv::Mat Matrix::copyToCV()
{
	cv::Mat result(rows, cols, CV_64FC(1));
	int index = 0;
	for (int i = 0; i < rows; i++) {
		for (int j = 0; j < cols; j++) {
			result.at<double>(i, j) = m[index];
			index++;
		}
	}
	return result;
}

Matrix operator+(const Matrix &A, const Matrix &B)
{
	if (A.getRows() != B.getRows() || A.getCols() != B.getCols()) {
        // AまたはBのどちらかが１次元行または列ベクトルの場合，ブロードキャストをする
        if (A.getRows() == 1 || B.getRows() == 1 || A.getCols() == 1 || B.getCols() == 1) {
            Matrix D;
            if (A.getRows() > 1 && A.getCols() > 1) {
                D = A;
                if (B.getRows() == 1) {
                    for (int i = 0; i < A.getRows(); i++) {
                        for (int j = 0; j < A.getCols(); j++) {
                            D(i, j) = B(0, j);
                        }
                    }
                } else if (B.getCols() == 1) {
                    for (int i = 0; i < A.getCols(); i++) {
                        for (int j = 0; j < A.getRows(); j++) {
                            D(j, i) = B(j, 0);
                        }
                    }
                }
                return A + D;
            } else {
                D = B;
                if (A.getRows() == 1) {
                    for (int i = 0; i < B.getRows(); i++) {
                        for (int j = 0; j < B.getCols(); j++) {
                            D(i, j) = A(0, j);
                        }
                    }
                } else if (A.getCols() == 1) {
                    for (int i = 0; i < B.getCols(); i++) {
                        for (int j = 0; j < B.getRows(); j++) {
                            D(j, i) = A(j, 0);
                        }
                    }
                }
                return B + D;
            }
        } else {
            std::cerr << "in Matrix operator + Error : 行列のサイズが異なります\n";
            exit(0);
        }
    }
    int row = A.getRows();
    int col = A.getCols();
    Matrix C(row, col);
    for (int r = 0; r < row; r++) {
        for (int c = 0; c < col; c++) {
            C(r, c) = A(r, c) + B(r, c);
        }
    }
    return C;
}

Matrix operator-(const Matrix &A, const Matrix &B)
{
	if (A.getRows() != B.getRows() || A.getCols() != B.getCols()) {
		std::cerr << "in Matrix operator - Error : 行列のサイズが異なります\n";
		exit(0);
	}
	int row = A.getRows();
	int col = A.getCols();
	Matrix C(row, col);
	for (int r = 0; r < row; r++) {
		for (int c = 0; c < col; c++) {
			C(r, c) = A(r, c) - B(r, c);
		}
	}
	return C;
}

Matrix operator*(const Matrix &A, const Matrix &B)
{
	// サイズチェック
	int col = A.getCols();
	int row = B.getRows();
	if (row != col) {
		std::cerr << "in Matrix operator * Error : 行列のサイズが異なります\n";
		exit(0);
	}

	int a_row = A.getRows();
	int b_col = B.getCols();
	Matrix C(a_row, b_col);
#pragma omp parallel for
	for (int r = 0; r < a_row; r++) {
        for (int k = 0; k < col; k++) {
            double tmp_a = A(r, k);
            for (int c = 0; c < b_col; c++) {
				C(r, c) += tmp_a * B(k, c);
				//std::cout << "C("<<r<<","<<c<<")+="<< *(A(r, k)) << "*" << *(B(k,c)) << std::endl;
			}
			//std::cout << std::endl;
		}
	}
	return C;
}

Matrix operator*(const double &k, Matrix &A)
{
	int row = A.getRows();
	int col = A.getCols();
	Matrix C(row, col);
	for (int r = 0; r < row; r++) {
		for (int c = 0; c < col; c++) {
			C(r, c) = A(r, c) * k;
		}
	}
	return C;
}

Matrix operator*(Matrix &A, const double &k)
{
	int row = A.getRows();
	int col = A.getCols();
	Matrix C(row, col);
	for (int r = 0; r < row; r++) {
		for (int c = 0; c < col; c++) {
			C(r, c) = A(r, c) * k;
		}
	}
	return C;
}

