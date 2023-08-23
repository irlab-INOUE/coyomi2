# 行列演算ライブラリ

Jan.31, 2019

Kazumichi INOUE <k.inoue@oyama-ct.ac.jp>

# Usage:

通常の（密な）行列はこちらの方法
```
Matrix A;	// 空の行列の定義
Matrix B(3, 3); 	// 3x3行列の定義, 要素は0
A = B.inv(); 		// AにBの逆行列を代入
Matrix C = A * B;	// 行列の積
Matrix D(3, 3);
C = D.I() * A.t() * B + D; 	// D.I()は単位行列，A.t()は転置行列
cv::Mat CVMat = C.copyToCV(); 	// cv::Mat へコピーする
```

疎行列はこちらの方法も使える
```
SparseMatrix A;	// 空の疎行列の定義
SparseMatrix B(3, 3); 	// 3x3疎行列の定義, 要素は空
A = B.inv(); 		// AにBの逆行列を代入，結果は疎行列形式
SparseMatrix C = A * B;	// 行列の積．疎行列形式
```


# Special Thanks

Kazuharu Nakajima and 1SE Members

中嶋氏からは参照返し，const 修飾子について極めて重要な貢献があった

Michael

マイケル氏が行列式，ベクトルの代入メソッドを作ってくれた！
