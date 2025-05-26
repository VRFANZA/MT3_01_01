#include <Novice.h>
#include <cmath>
#include <Vector3.h>
#include <Matrix4x4.h>
#include <assert.h>
#include <sstream>
#include <vector>
#include "SerialReader.h"

const char kWindowTitle[] = "LE2B_16_タカムラシュン_MT3_01_01";

const float kWindowWidth = 1080.0f;
const float kWindowHeight = 720.0f;

//===================================
// MT3でも使う関数の宣言
//===================================

Vector3 Cross(const Vector3& v1, const Vector3& v2);

// 行列をベクトルに変換する関数
Vector3 Transform(const Vector3& vector, const Matrix4x4& matrix);

/// <summary>
/// cotangent(余接)を求める関数
/// </summary>
/// <param name="theta">θ(シータ)</param>
/// <returns>cotangent</returns>
float Cotangent(float theta);

/// <summary>
/// 4x4行列の積を求める関数
/// </summary>
/// <param name="matrix1">1つ目の行列</param>
/// <param name="matrix2">1つ目の行列</param>
/// <returns>4x4行列の積</returns>
Matrix4x4 Multiply(Matrix4x4 matrix1, Matrix4x4 matrix2);

/// <summary>
/// 4x4単位行列作成関数
/// </summary>
/// <returns>4x4単位行列</returns>
Matrix4x4 MakeIdentity4x4();

/// <summary>
/// アフィン行列作成関数
/// </summary>
/// <param name="scale">縮尺</param>
/// <param name="rotate">thetaを求めるための数値</param>
/// <param name="translate">三次元座標でのx,y,zの移動量</param>
/// <returns>アフィン行列</returns>
Matrix4x4 MakeAffineMatrix(Vector3 scale, Vector3 rotate, Vector3 translate);

/// <summary>
/// 投視投影行列作成関数
/// </summary>
/// <param name="fovY">縦の画角</param>
/// <param name="aspectRatio">アスペクト比</param>
/// <param name="nearClip">近平面への距離</param>
/// <param name="farClip">遠平面への距離</param>
/// <returns>投視投影行列</returns>
Matrix4x4 MakePerspectiveFovMatrix(float fovY, float aspectRatio, float nearClip, float farClip);

/// <summary>
/// 正射影行列作関数
/// </summary>
/// <param name="left">左側の座標</param>
/// <param name="top">上側の座標</param>
/// <param name="right">右側の座標</param>
/// <param name="bottom">下側の座標</param>
/// <param name="nearClip">近平面への距離</param>
/// <param name="farClip">遠平面への距離</param>
/// <returns>正射影行列</returns>
Matrix4x4 MakeOrthographicMatrix(float left, float top, float right, float bottom, float nearClip, float farClip);

/// <summary>
/// 4x4逆行列を求める関数
/// </summary>
/// <param name="matrix4x4">逆行列を求めたい行列</param>
/// <returns>4x4逆行列</returns>
Matrix4x4 Inverse(Matrix4x4 matrix4x4);

/// <summary>
/// ビューポート行列作成関数
/// </summary>
/// <param name="left">左側の座標</param>
/// <param name="top">上側の座標</param>
/// <param name="width">切り取るスクリーンの幅</param>
/// <param name="height">切り取るスクリーンの高さ</param>
/// <param name="minDepth">最小深度値</param>
/// <param name="maxDepth">最大深度値</param>
/// <returns></returns>
Matrix4x4 MakeViewportMatrix(float left, float top, float width, float height, float minDepth, float maxDepth);


// Windowsアプリでのエントリーポイント(main関数)
int WINAPI WinMain(HINSTANCE, HINSTANCE, LPSTR, int) {

	// ライブラリの初期化
	Novice::Initialize(kWindowTitle, 1280, 720);

	// キー入力結果を受け取る箱
	char keys[256] = { 0 };
	char preKeys[256] = { 0 };

	// グローバル or 上部で宣言
	SerialReader* serial = nullptr;
	float sensorData[10] = { 0.0f,0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };

	// WinMain の初期化部分
	serial = new SerialReader("COM4");  // ArduinoのCOMポート

	Vector3 rotate{ 0.0f,0.0f,0.0f };
	Vector3 translate{ 0.0f,0.0f,0.0f };

	// ウィンドウの×ボタンが押されるまでループ
	while (Novice::ProcessMessage() == 0) {
		// フレームの開始
		Novice::BeginFrame();

		// キー入力を受け取る
		memcpy(preKeys, keys, 256);
		Novice::GetHitKeyStateAll(keys);

		///
		/// ↓更新処理ここから
		///

		//	センサーデータを取得して格納
		std::string line;
		if (serial->readLine(line)) {
			std::istringstream ss(line);
			std::string token;
			for (int i = 0; std::getline(ss, token, ',') && i < 10; ++i) {
				sensorData[i] = std::stof(token);
			}
		}

		// ここに操作を記述
		if (keys[DIK_W]) {
			translate.z += 0.1f;
		}

		if (keys[DIK_S]) {
			translate.z -= 0.1f;
		}

		if (keys[DIK_A]) {
			translate.x -= 0.1f;
		}

		if (keys[DIK_D]) {
			translate.x += 0.1f;
		}

		// センサーの角度（degree）をradianに変換してVector3に反映
		// 60で割ってるのは6.0fで1回転で数値をそのまま入れるとめっちゃ回転するから
		rotate = {
			(sensorData[2] * (3.141592f / 180.0f)) / 60.0f, // pitch → X
			(sensorData[1] + 180.0f * (3.141592f / 180.0f)) / 60.0f, // heading → Y
			(sensorData[3] + 180.0f * (3.141592f / 180.0f)) / 60.0f  // roll → Z
		};

		Matrix4x4 worldMatrix = MakeAffineMatrix({ 1.0f, 1.0f, 1.0f }, rotate, translate);

		Matrix4x4 cameraMatrix =
			MakeAffineMatrix({ 1.0f,1.0f,1.0f }, { 0.0f,0.0f,0.0f }, { 0.0f,0.0f,-10.0f });

		Matrix4x4 viewMatrix =
			Inverse(cameraMatrix);

		Matrix4x4 projectionMatrix =
			MakePerspectiveFovMatrix(0.45f, kWindowWidth / kWindowHeight, 0.1f, 100.0f);

		Matrix4x4 worldViewProjectionMatrix =
			Multiply(worldMatrix, Multiply(viewMatrix, projectionMatrix));

		Matrix4x4 viewportMatrix =
			MakeViewportMatrix(0, 0, kWindowWidth, kWindowHeight, 0.0f, 1.0f);

		Vector3 screenVertices[3];
		Vector3 kLocalVertices[3] = { {0.0f,1.0f,0.0f},{1.0f,-1.0f,0.0f},{-1.0f,-1.0f,0.0f} };
		for (uint32_t i = 0; i < 3; ++i) {
			Vector3 ndcVertex = Transform(kLocalVertices[i], worldViewProjectionMatrix);
			screenVertices[i] = Transform(ndcVertex, viewportMatrix);
		}

		///
		/// ↑更新処理ここまで
		///

		///
		/// ↓描画処理ここから
		///

		Novice::DrawTriangle(
			static_cast<int>(screenVertices[0].x),
			static_cast<int>(screenVertices[0].y),
			static_cast<int>(screenVertices[1].x),
			static_cast<int>(screenVertices[1].y),
			static_cast<int>(screenVertices[2].x),
			static_cast<int>(screenVertices[2].y),
			RED, kFillModeSolid
		);

		// センサー取得地の値を画面に描画する（左上に表示）
		Novice::ScreenPrintf(10, 10, "Pitch(x): %f", sensorData[2]);
		Novice::ScreenPrintf(10, 30, "Heading(y): %f", sensorData[1]);
		Novice::ScreenPrintf(10, 50, "Roll(z):  %f", sensorData[3]);

		///
		/// ↑描画処理ここまで
		///

		// フレームの終了
		Novice::EndFrame();

		// ESCキーが押されたらループを抜ける
		if (preKeys[DIK_ESCAPE] == 0 && keys[DIK_ESCAPE] != 0) {
			break;
		}
	}

	// ライブラリの終了
	Novice::Finalize();
	return 0;
}
//===============================
// MT3でも使う関数
//===============================
float Cotangent(float theta)
{
	float cotngent;

	cotngent = 1.0f / std::tanf(theta);

	return cotngent;
}

Vector3 Cross(const Vector3& v1, const Vector3& v2)
{
	Vector3 result;

	result.x = (v1.y * v2.z) - (v1.z * v2.y);
	result.x = (v1.z * v2.x) - (v1.x * v2.z);
	result.x = (v1.x * v2.y) - (v1.y * v2.x);

	return result;
}

Vector3 Transform(const Vector3& vector, const Matrix4x4& matrix)
{
	Vector3 resultVector3;

	resultVector3.x = vector.x * matrix.m[0][0] + vector.y * matrix.m[1][0] + vector.z * matrix.m[2][0] + 1.0f * matrix.m[3][0];
	resultVector3.y = vector.x * matrix.m[0][1] + vector.y * matrix.m[1][1] + vector.z * matrix.m[2][1] + 1.0f * matrix.m[3][1];
	resultVector3.z = vector.x * matrix.m[0][2] + vector.y * matrix.m[1][2] + vector.z * matrix.m[2][2] + 1.0f * matrix.m[3][2];

	float w = vector.x * matrix.m[0][3] + vector.y * matrix.m[1][3] + vector.z * matrix.m[2][3] + 1.0f * matrix.m[3][3];

	assert(w != 0.0f);

	resultVector3.x /= w;
	resultVector3.y /= w;
	resultVector3.z /= w;

	return resultVector3;
}

Matrix4x4 Multiply(Matrix4x4 matrix1, Matrix4x4 matrix2)
{
	Matrix4x4 resoultMatrix4x4;

	resoultMatrix4x4.m[0][0] = matrix1.m[0][0] * matrix2.m[0][0] + matrix1.m[0][1] * matrix2.m[1][0] + matrix1.m[0][2] * matrix2.m[2][0] + matrix1.m[0][3] * matrix2.m[3][0];
	resoultMatrix4x4.m[0][1] = matrix1.m[0][0] * matrix2.m[0][1] + matrix1.m[0][1] * matrix2.m[1][1] + matrix1.m[0][2] * matrix2.m[2][1] + matrix1.m[0][3] * matrix2.m[3][1];
	resoultMatrix4x4.m[0][2] = matrix1.m[0][0] * matrix2.m[0][2] + matrix1.m[0][1] * matrix2.m[1][2] + matrix1.m[0][2] * matrix2.m[2][2] + matrix1.m[0][3] * matrix2.m[3][2];
	resoultMatrix4x4.m[0][3] = matrix1.m[0][0] * matrix2.m[0][3] + matrix1.m[0][1] * matrix2.m[1][3] + matrix1.m[0][2] * matrix2.m[2][3] + matrix1.m[0][3] * matrix2.m[3][3];

	resoultMatrix4x4.m[1][0] = matrix1.m[1][0] * matrix2.m[0][0] + matrix1.m[1][1] * matrix2.m[1][0] + matrix1.m[1][2] * matrix2.m[2][0] + matrix1.m[1][3] * matrix2.m[3][0];
	resoultMatrix4x4.m[1][1] = matrix1.m[1][0] * matrix2.m[0][1] + matrix1.m[1][1] * matrix2.m[1][1] + matrix1.m[1][2] * matrix2.m[2][1] + matrix1.m[1][3] * matrix2.m[3][1];
	resoultMatrix4x4.m[1][2] = matrix1.m[1][0] * matrix2.m[0][2] + matrix1.m[1][1] * matrix2.m[1][2] + matrix1.m[1][2] * matrix2.m[2][2] + matrix1.m[1][3] * matrix2.m[3][2];
	resoultMatrix4x4.m[1][3] = matrix1.m[1][0] * matrix2.m[0][3] + matrix1.m[1][1] * matrix2.m[1][3] + matrix1.m[1][2] * matrix2.m[2][3] + matrix1.m[1][3] * matrix2.m[3][3];

	resoultMatrix4x4.m[2][0] = matrix1.m[2][0] * matrix2.m[0][0] + matrix1.m[2][1] * matrix2.m[1][0] + matrix1.m[2][2] * matrix2.m[2][0] + matrix1.m[2][3] * matrix2.m[3][0];
	resoultMatrix4x4.m[2][1] = matrix1.m[2][0] * matrix2.m[0][1] + matrix1.m[2][1] * matrix2.m[1][1] + matrix1.m[2][2] * matrix2.m[2][1] + matrix1.m[2][3] * matrix2.m[3][1];
	resoultMatrix4x4.m[2][2] = matrix1.m[2][0] * matrix2.m[0][2] + matrix1.m[2][1] * matrix2.m[1][2] + matrix1.m[2][2] * matrix2.m[2][2] + matrix1.m[2][3] * matrix2.m[3][2];
	resoultMatrix4x4.m[2][3] = matrix1.m[2][0] * matrix2.m[0][3] + matrix1.m[2][1] * matrix2.m[1][3] + matrix1.m[2][2] * matrix2.m[2][3] + matrix1.m[2][3] * matrix2.m[3][3];

	resoultMatrix4x4.m[3][0] = matrix1.m[3][0] * matrix2.m[0][0] + matrix1.m[3][1] * matrix2.m[1][0] + matrix1.m[3][2] * matrix2.m[2][0] + matrix1.m[3][3] * matrix2.m[3][0];
	resoultMatrix4x4.m[3][1] = matrix1.m[3][0] * matrix2.m[0][1] + matrix1.m[3][1] * matrix2.m[1][1] + matrix1.m[3][2] * matrix2.m[2][1] + matrix1.m[3][3] * matrix2.m[3][1];
	resoultMatrix4x4.m[3][2] = matrix1.m[3][0] * matrix2.m[0][2] + matrix1.m[3][1] * matrix2.m[1][2] + matrix1.m[3][2] * matrix2.m[2][2] + matrix1.m[3][3] * matrix2.m[3][2];
	resoultMatrix4x4.m[3][3] = matrix1.m[3][0] * matrix2.m[0][3] + matrix1.m[3][1] * matrix2.m[1][3] + matrix1.m[3][2] * matrix2.m[2][3] + matrix1.m[3][3] * matrix2.m[3][3];

	return resoultMatrix4x4;
}

Matrix4x4 MakeIdentity4x4()
{
	Matrix4x4 identityMatrix4x4;

	identityMatrix4x4.m[0][0] = 1.0f;
	identityMatrix4x4.m[0][1] = 0.0f;
	identityMatrix4x4.m[0][2] = 0.0f;
	identityMatrix4x4.m[0][3] = 0.0f;
	identityMatrix4x4.m[1][0] = 0.0f;
	identityMatrix4x4.m[1][1] = 1.0f;
	identityMatrix4x4.m[1][2] = 0.0f;
	identityMatrix4x4.m[1][3] = 0.0f;
	identityMatrix4x4.m[2][0] = 0.0f;
	identityMatrix4x4.m[2][1] = 0.0f;
	identityMatrix4x4.m[2][2] = 1.0f;
	identityMatrix4x4.m[2][3] = 0.0f;
	identityMatrix4x4.m[3][0] = 0.0f;
	identityMatrix4x4.m[3][1] = 0.0f;
	identityMatrix4x4.m[3][2] = 0.0f;
	identityMatrix4x4.m[3][3] = 1.0f;

	return identityMatrix4x4;
}



Matrix4x4 MakeAffineMatrix(Vector3 scale, Vector3 rotate, Vector3 translate)
{
	//====================
	// 拡縮の行列の作成
	//====================
	Matrix4x4 scaleMatrix4x4;
	scaleMatrix4x4.m[0][0] = scale.x;
	scaleMatrix4x4.m[0][1] = 0.0f;
	scaleMatrix4x4.m[0][2] = 0.0f;
	scaleMatrix4x4.m[0][3] = 0.0f;

	scaleMatrix4x4.m[1][0] = 0.0f;
	scaleMatrix4x4.m[1][1] = scale.y;
	scaleMatrix4x4.m[1][2] = 0.0f;
	scaleMatrix4x4.m[1][3] = 0.0f;

	scaleMatrix4x4.m[2][0] = 0.0f;
	scaleMatrix4x4.m[2][1] = 0.0f;
	scaleMatrix4x4.m[2][2] = scale.z;
	scaleMatrix4x4.m[2][3] = 0.0f;

	scaleMatrix4x4.m[3][0] = 0.0f;
	scaleMatrix4x4.m[3][1] = 0.0f;
	scaleMatrix4x4.m[3][2] = 0.0f;
	scaleMatrix4x4.m[3][3] = 1.0f;

	//===================
	// 回転の行列の作成
	//===================
	// Xの回転行列
	Matrix4x4 rotateMatrixX;
	rotateMatrixX.m[0][0] = 1.0f;
	rotateMatrixX.m[0][1] = 0.0f;
	rotateMatrixX.m[0][2] = 0.0f;
	rotateMatrixX.m[0][3] = 0.0f;

	rotateMatrixX.m[1][0] = 0.0f;
	rotateMatrixX.m[1][1] = cosf(rotate.x);
	rotateMatrixX.m[1][2] = sinf(rotate.x);
	rotateMatrixX.m[1][3] = 0.0f;

	rotateMatrixX.m[2][0] = 0.0f;
	rotateMatrixX.m[2][1] = -sinf(rotate.x);
	rotateMatrixX.m[2][2] = cosf(rotate.x);
	rotateMatrixX.m[2][3] = 0.0f;

	rotateMatrixX.m[3][0] = 0.0f;
	rotateMatrixX.m[3][1] = 0.0f;
	rotateMatrixX.m[3][2] = 0.0f;
	rotateMatrixX.m[3][3] = 1.0f;

	// Yの回転行列
	Matrix4x4 rotateMatrixY;
	rotateMatrixY.m[0][0] = cosf(rotate.y);
	rotateMatrixY.m[0][1] = 0.0f;
	rotateMatrixY.m[0][2] = -sinf(rotate.y);
	rotateMatrixY.m[0][3] = 0.0f;

	rotateMatrixY.m[1][0] = 0.0f;
	rotateMatrixY.m[1][1] = 1.0f;
	rotateMatrixY.m[1][2] = 0.0f;
	rotateMatrixY.m[1][3] = 0.0f;

	rotateMatrixY.m[2][0] = sinf(rotate.y);
	rotateMatrixY.m[2][1] = 0.0f;
	rotateMatrixY.m[2][2] = cosf(rotate.y);
	rotateMatrixY.m[2][3] = 0.0f;

	rotateMatrixY.m[3][0] = 0.0f;
	rotateMatrixY.m[3][1] = 0.0f;
	rotateMatrixY.m[3][2] = 0.0f;
	rotateMatrixY.m[3][3] = 1.0f;

	// Zの回転行列
	Matrix4x4 rotateMatrixZ;
	rotateMatrixZ.m[0][0] = cosf(rotate.z);
	rotateMatrixZ.m[0][1] = sinf(rotate.z);
	rotateMatrixZ.m[0][2] = 0.0f;
	rotateMatrixZ.m[0][3] = 0.0f;

	rotateMatrixZ.m[1][0] = -sinf(rotate.z);
	rotateMatrixZ.m[1][1] = cosf(rotate.z);
	rotateMatrixZ.m[1][2] = 0.0f;
	rotateMatrixZ.m[1][3] = 0.0f;

	rotateMatrixZ.m[2][0] = 0.0f;
	rotateMatrixZ.m[2][1] = 0.0f;
	rotateMatrixZ.m[2][2] = 1.0f;
	rotateMatrixZ.m[2][3] = 0.0f;

	rotateMatrixZ.m[3][0] = 0.0f;
	rotateMatrixZ.m[3][1] = 0.0f;
	rotateMatrixZ.m[3][2] = 0.0f;
	rotateMatrixZ.m[3][3] = 1.0f;

	// 回転行列の作成
	Matrix4x4 rotateMatrix4x4;

	rotateMatrix4x4 = Multiply(rotateMatrixX, Multiply(rotateMatrixY, rotateMatrixZ));

	//==================
	// 移動の行列の作成
	//==================
	Matrix4x4 translateMatrix4x4;
	translateMatrix4x4.m[0][0] = 1.0f;
	translateMatrix4x4.m[0][1] = 0.0f;
	translateMatrix4x4.m[0][2] = 0.0f;
	translateMatrix4x4.m[0][3] = 0.0f;

	translateMatrix4x4.m[1][0] = 0.0f;
	translateMatrix4x4.m[1][1] = 1.0f;
	translateMatrix4x4.m[1][2] = 0.0f;
	translateMatrix4x4.m[1][3] = 0.0f;

	translateMatrix4x4.m[2][0] = 0.0f;
	translateMatrix4x4.m[2][1] = 0.0f;
	translateMatrix4x4.m[2][2] = 1.0f;
	translateMatrix4x4.m[2][3] = 0.0f;

	translateMatrix4x4.m[3][0] = translate.x;
	translateMatrix4x4.m[3][1] = translate.y;
	translateMatrix4x4.m[3][2] = translate.z;
	translateMatrix4x4.m[3][3] = 1.0f;

	//====================
	// アフィン行列の作成
	//====================
	// 上で作った行列からアフィン行列を作る
	// アフィン行列の作成（スケール→回転→移動の順）
	Matrix4x4 affineMatrix4x4;
	affineMatrix4x4 = Multiply(scaleMatrix4x4, Multiply(rotateMatrix4x4, translateMatrix4x4));

	return  affineMatrix4x4;
}

Matrix4x4 MakePerspectiveFovMatrix(float fovY, float aspectRatio, float nearClip, float farClip)
{
	Matrix4x4 perspectiveFovMatrix;

	perspectiveFovMatrix.m[0][0] = 1.0f / aspectRatio * Cotangent(fovY / 2.0f);
	perspectiveFovMatrix.m[0][1] = 0.0f;
	perspectiveFovMatrix.m[0][2] = 0.0f;
	perspectiveFovMatrix.m[0][3] = 0.0f;

	perspectiveFovMatrix.m[1][0] = 0.0f;
	perspectiveFovMatrix.m[1][1] = Cotangent(fovY / 2.0f);
	perspectiveFovMatrix.m[1][2] = 0.0f;
	perspectiveFovMatrix.m[1][3] = 0.0f;

	perspectiveFovMatrix.m[2][0] = 0.0f;
	perspectiveFovMatrix.m[2][1] = 0.0f;
	perspectiveFovMatrix.m[2][2] = farClip / (farClip - nearClip);
	perspectiveFovMatrix.m[2][3] = 1.0f;

	perspectiveFovMatrix.m[3][0] = 0.0f;
	perspectiveFovMatrix.m[3][1] = 0.0f;
	perspectiveFovMatrix.m[3][2] = (-nearClip * farClip) / (farClip - nearClip);
	perspectiveFovMatrix.m[3][3] = 0.0f;

	return perspectiveFovMatrix;
}

Matrix4x4 MakeOrthographicMatrix(float left, float top, float right, float bottom, float nearClip, float farClip)
{
	Matrix4x4 orthographicMatrix;

	orthographicMatrix.m[0][0] = 2.0f / (right - left);
	orthographicMatrix.m[0][1] = 0.0f;
	orthographicMatrix.m[0][2] = 0.0f;
	orthographicMatrix.m[0][3] = 0.0f;

	orthographicMatrix.m[1][0] = 0.0f;
	orthographicMatrix.m[1][1] = 2.0f / (top - bottom);
	orthographicMatrix.m[1][2] = 0.0f;
	orthographicMatrix.m[1][3] = 0.0f;

	orthographicMatrix.m[2][0] = 0.0f;
	orthographicMatrix.m[2][1] = 0.0f;
	orthographicMatrix.m[2][2] = 1.0f / (farClip - nearClip);
	orthographicMatrix.m[2][3] = 0.0f;

	orthographicMatrix.m[3][0] = (left + right) / (left - right);
	orthographicMatrix.m[3][1] = (top + bottom) / (bottom - top);
	orthographicMatrix.m[3][2] = (nearClip) / (nearClip - farClip);
	orthographicMatrix.m[3][3] = 1.0f;

	return orthographicMatrix;
}

Matrix4x4 Inverse(Matrix4x4 matrix4x4)
{
	// 行列式|A|を求める
	float bottom =
		(matrix4x4.m[0][0] * matrix4x4.m[1][1] * matrix4x4.m[2][2] * matrix4x4.m[3][3])
		+ (matrix4x4.m[0][0] * matrix4x4.m[1][2] * matrix4x4.m[2][3] * matrix4x4.m[3][1])
		+ (matrix4x4.m[0][0] * matrix4x4.m[1][3] * matrix4x4.m[2][1] * matrix4x4.m[3][2])
		- (matrix4x4.m[0][0] * matrix4x4.m[1][3] * matrix4x4.m[2][2] * matrix4x4.m[3][1])
		- (matrix4x4.m[0][0] * matrix4x4.m[1][2] * matrix4x4.m[2][1] * matrix4x4.m[3][3])
		- (matrix4x4.m[0][0] * matrix4x4.m[1][1] * matrix4x4.m[2][3] * matrix4x4.m[3][2])
		- (matrix4x4.m[0][1] * matrix4x4.m[1][0] * matrix4x4.m[2][2] * matrix4x4.m[3][3])
		- (matrix4x4.m[0][2] * matrix4x4.m[1][0] * matrix4x4.m[2][3] * matrix4x4.m[3][1])
		- (matrix4x4.m[0][3] * matrix4x4.m[1][0] * matrix4x4.m[2][1] * matrix4x4.m[3][2])
		+ (matrix4x4.m[0][3] * matrix4x4.m[1][0] * matrix4x4.m[2][2] * matrix4x4.m[3][1])
		+ (matrix4x4.m[0][2] * matrix4x4.m[1][0] * matrix4x4.m[2][1] * matrix4x4.m[3][3])
		+ (matrix4x4.m[0][1] * matrix4x4.m[1][0] * matrix4x4.m[2][3] * matrix4x4.m[3][2])
		+ (matrix4x4.m[0][1] * matrix4x4.m[1][2] * matrix4x4.m[2][0] * matrix4x4.m[3][3])
		+ (matrix4x4.m[0][2] * matrix4x4.m[1][3] * matrix4x4.m[2][0] * matrix4x4.m[3][1])
		+ (matrix4x4.m[0][3] * matrix4x4.m[1][1] * matrix4x4.m[2][0] * matrix4x4.m[3][2])
		- (matrix4x4.m[0][3] * matrix4x4.m[1][2] * matrix4x4.m[2][0] * matrix4x4.m[3][1])
		- (matrix4x4.m[0][2] * matrix4x4.m[1][1] * matrix4x4.m[2][0] * matrix4x4.m[3][3])
		- (matrix4x4.m[0][1] * matrix4x4.m[1][3] * matrix4x4.m[2][0] * matrix4x4.m[3][2])
		- (matrix4x4.m[0][1] * matrix4x4.m[1][2] * matrix4x4.m[2][3] * matrix4x4.m[3][0])
		- (matrix4x4.m[0][2] * matrix4x4.m[1][3] * matrix4x4.m[2][1] * matrix4x4.m[3][0])
		- (matrix4x4.m[0][3] * matrix4x4.m[1][1] * matrix4x4.m[2][2] * matrix4x4.m[3][0])
		+ (matrix4x4.m[0][3] * matrix4x4.m[1][2] * matrix4x4.m[2][1] * matrix4x4.m[3][0])
		+ (matrix4x4.m[0][2] * matrix4x4.m[1][1] * matrix4x4.m[2][3] * matrix4x4.m[3][0])
		+ (matrix4x4.m[0][1] * matrix4x4.m[1][3] * matrix4x4.m[2][2] * matrix4x4.m[3][0]);

	Matrix4x4 resoultMatrix;

	// 1行目
	resoultMatrix.m[0][0] = 1.0f / bottom * (
		(matrix4x4.m[1][1] * matrix4x4.m[2][2] * matrix4x4.m[3][3])
		+ (matrix4x4.m[1][2] * matrix4x4.m[2][3] * matrix4x4.m[3][1])
		+ (matrix4x4.m[1][3] * matrix4x4.m[2][1] * matrix4x4.m[3][2])
		- (matrix4x4.m[1][3] * matrix4x4.m[2][2] * matrix4x4.m[3][1])
		- (matrix4x4.m[1][2] * matrix4x4.m[2][1] * matrix4x4.m[3][3])
		- (matrix4x4.m[1][1] * matrix4x4.m[2][3] * matrix4x4.m[3][2]));

	resoultMatrix.m[0][1] = 1.0f / bottom * (
		-(matrix4x4.m[0][1] * matrix4x4.m[2][2] * matrix4x4.m[3][3])
		- (matrix4x4.m[0][2] * matrix4x4.m[2][3] * matrix4x4.m[3][1])
		- (matrix4x4.m[0][3] * matrix4x4.m[2][1] * matrix4x4.m[3][2])
		+ (matrix4x4.m[0][3] * matrix4x4.m[2][2] * matrix4x4.m[3][1])
		+ (matrix4x4.m[0][2] * matrix4x4.m[2][1] * matrix4x4.m[3][3])
		+ (matrix4x4.m[0][1] * matrix4x4.m[2][3] * matrix4x4.m[3][2]));

	resoultMatrix.m[0][2] = 1.0f / bottom * (
		(matrix4x4.m[0][1] * matrix4x4.m[1][2] * matrix4x4.m[3][3])
		+ (matrix4x4.m[0][2] * matrix4x4.m[1][3] * matrix4x4.m[3][1])
		+ (matrix4x4.m[0][3] * matrix4x4.m[1][1] * matrix4x4.m[3][2])
		- (matrix4x4.m[0][3] * matrix4x4.m[1][2] * matrix4x4.m[3][1])
		- (matrix4x4.m[0][2] * matrix4x4.m[1][1] * matrix4x4.m[3][3])
		- (matrix4x4.m[0][1] * matrix4x4.m[1][3] * matrix4x4.m[3][2]));

	resoultMatrix.m[0][3] = 1.0f / bottom * (
		-(matrix4x4.m[0][1] * matrix4x4.m[1][2] * matrix4x4.m[2][3])
		- (matrix4x4.m[0][2] * matrix4x4.m[1][3] * matrix4x4.m[2][1])
		- (matrix4x4.m[0][3] * matrix4x4.m[1][1] * matrix4x4.m[2][2])
		+ (matrix4x4.m[0][3] * matrix4x4.m[1][2] * matrix4x4.m[2][1])
		+ (matrix4x4.m[0][2] * matrix4x4.m[1][1] * matrix4x4.m[2][3])
		+ (matrix4x4.m[0][1] * matrix4x4.m[1][3] * matrix4x4.m[2][2]));

	// 2行目
	resoultMatrix.m[1][0] = 1.0f / bottom * (
		-(matrix4x4.m[1][0] * matrix4x4.m[2][2] * matrix4x4.m[3][3])
		- (matrix4x4.m[1][2] * matrix4x4.m[2][3] * matrix4x4.m[3][0])
		- (matrix4x4.m[1][3] * matrix4x4.m[2][0] * matrix4x4.m[3][2])
		+ (matrix4x4.m[1][3] * matrix4x4.m[2][2] * matrix4x4.m[3][0])
		+ (matrix4x4.m[1][2] * matrix4x4.m[2][0] * matrix4x4.m[3][3])
		+ (matrix4x4.m[1][0] * matrix4x4.m[2][3] * matrix4x4.m[3][2]));

	resoultMatrix.m[1][1] = 1.0f / bottom * (
		(matrix4x4.m[0][0] * matrix4x4.m[2][2] * matrix4x4.m[3][3])
		+ (matrix4x4.m[0][2] * matrix4x4.m[2][3] * matrix4x4.m[3][0])
		+ (matrix4x4.m[0][3] * matrix4x4.m[2][0] * matrix4x4.m[3][2])
		- (matrix4x4.m[0][3] * matrix4x4.m[2][2] * matrix4x4.m[3][0])
		- (matrix4x4.m[0][2] * matrix4x4.m[2][0] * matrix4x4.m[3][3])
		- (matrix4x4.m[0][0] * matrix4x4.m[2][3] * matrix4x4.m[3][2]));

	resoultMatrix.m[1][2] = 1.0f / bottom * (
		-(matrix4x4.m[0][0] * matrix4x4.m[1][2] * matrix4x4.m[3][3])
		- (matrix4x4.m[0][2] * matrix4x4.m[1][3] * matrix4x4.m[3][0])
		- (matrix4x4.m[0][3] * matrix4x4.m[1][0] * matrix4x4.m[3][2])
		+ (matrix4x4.m[0][3] * matrix4x4.m[1][2] * matrix4x4.m[3][0])
		+ (matrix4x4.m[0][2] * matrix4x4.m[1][0] * matrix4x4.m[3][3])
		+ (matrix4x4.m[0][0] * matrix4x4.m[1][3] * matrix4x4.m[3][2]));

	resoultMatrix.m[1][3] = 1.0f / bottom * (
		(matrix4x4.m[0][0] * matrix4x4.m[1][2] * matrix4x4.m[2][3])
		+ (matrix4x4.m[0][2] * matrix4x4.m[1][3] * matrix4x4.m[2][0])
		+ (matrix4x4.m[0][3] * matrix4x4.m[1][0] * matrix4x4.m[2][2])
		- (matrix4x4.m[0][3] * matrix4x4.m[1][2] * matrix4x4.m[2][0])
		- (matrix4x4.m[0][2] * matrix4x4.m[1][0] * matrix4x4.m[2][3])
		- (matrix4x4.m[0][0] * matrix4x4.m[1][3] * matrix4x4.m[2][2]));

	// 3行目
	resoultMatrix.m[2][0] = 1.0f / bottom * (
		(matrix4x4.m[1][0] * matrix4x4.m[2][1] * matrix4x4.m[3][3])
		+ (matrix4x4.m[1][1] * matrix4x4.m[2][3] * matrix4x4.m[3][0])
		+ (matrix4x4.m[1][3] * matrix4x4.m[2][0] * matrix4x4.m[3][1])
		- (matrix4x4.m[1][3] * matrix4x4.m[2][1] * matrix4x4.m[3][0])
		- (matrix4x4.m[1][1] * matrix4x4.m[2][0] * matrix4x4.m[3][3])
		- (matrix4x4.m[1][0] * matrix4x4.m[2][3] * matrix4x4.m[3][1]));

	resoultMatrix.m[2][1] = 1.0f / bottom * (
		-(matrix4x4.m[0][0] * matrix4x4.m[2][1] * matrix4x4.m[3][3])
		- (matrix4x4.m[0][1] * matrix4x4.m[2][3] * matrix4x4.m[3][0])
		- (matrix4x4.m[0][3] * matrix4x4.m[2][0] * matrix4x4.m[3][1])
		+ (matrix4x4.m[0][3] * matrix4x4.m[2][1] * matrix4x4.m[3][0])
		+ (matrix4x4.m[0][1] * matrix4x4.m[2][0] * matrix4x4.m[3][3])
		+ (matrix4x4.m[0][0] * matrix4x4.m[2][3] * matrix4x4.m[3][1]));

	resoultMatrix.m[2][2] = 1.0f / bottom * (
		(matrix4x4.m[0][0] * matrix4x4.m[1][1] * matrix4x4.m[3][3])
		+ (matrix4x4.m[0][1] * matrix4x4.m[1][3] * matrix4x4.m[3][0])
		+ (matrix4x4.m[0][3] * matrix4x4.m[1][0] * matrix4x4.m[3][1])
		- (matrix4x4.m[0][3] * matrix4x4.m[1][1] * matrix4x4.m[3][0])
		- (matrix4x4.m[0][1] * matrix4x4.m[1][0] * matrix4x4.m[3][3])
		- (matrix4x4.m[0][0] * matrix4x4.m[1][3] * matrix4x4.m[3][1]));

	resoultMatrix.m[2][3] = 1.0f / bottom * (
		-(matrix4x4.m[0][0] * matrix4x4.m[1][1] * matrix4x4.m[2][3])
		- (matrix4x4.m[0][1] * matrix4x4.m[1][3] * matrix4x4.m[2][0])
		- (matrix4x4.m[0][3] * matrix4x4.m[1][0] * matrix4x4.m[2][1])
		+ (matrix4x4.m[0][3] * matrix4x4.m[1][1] * matrix4x4.m[2][0])
		+ (matrix4x4.m[0][1] * matrix4x4.m[1][0] * matrix4x4.m[2][3])
		+ (matrix4x4.m[0][0] * matrix4x4.m[1][3] * matrix4x4.m[2][1]));

	// 4行目
	resoultMatrix.m[3][0] = 1.0f / bottom * (
		-(matrix4x4.m[1][0] * matrix4x4.m[2][1] * matrix4x4.m[3][2])
		- (matrix4x4.m[1][1] * matrix4x4.m[2][2] * matrix4x4.m[3][0])
		- (matrix4x4.m[1][2] * matrix4x4.m[2][0] * matrix4x4.m[3][1])
		+ (matrix4x4.m[1][2] * matrix4x4.m[2][1] * matrix4x4.m[3][0])
		+ (matrix4x4.m[1][1] * matrix4x4.m[2][0] * matrix4x4.m[3][2])
		+ (matrix4x4.m[1][0] * matrix4x4.m[2][2] * matrix4x4.m[3][1]));

	resoultMatrix.m[3][1] = 1.0f / bottom * (
		(matrix4x4.m[0][0] * matrix4x4.m[2][1] * matrix4x4.m[3][2])
		+ (matrix4x4.m[0][1] * matrix4x4.m[2][2] * matrix4x4.m[3][0])
		+ (matrix4x4.m[0][2] * matrix4x4.m[2][0] * matrix4x4.m[3][1])
		- (matrix4x4.m[0][2] * matrix4x4.m[2][1] * matrix4x4.m[3][0])
		- (matrix4x4.m[0][1] * matrix4x4.m[2][0] * matrix4x4.m[3][2])
		- (matrix4x4.m[0][0] * matrix4x4.m[2][2] * matrix4x4.m[3][1]));

	resoultMatrix.m[3][2] = 1.0f / bottom * (
		-(matrix4x4.m[0][0] * matrix4x4.m[1][1] * matrix4x4.m[3][2])
		- (matrix4x4.m[0][1] * matrix4x4.m[1][2] * matrix4x4.m[3][0])
		- (matrix4x4.m[0][2] * matrix4x4.m[1][0] * matrix4x4.m[3][1])
		+ (matrix4x4.m[0][2] * matrix4x4.m[1][1] * matrix4x4.m[3][0])
		+ (matrix4x4.m[0][1] * matrix4x4.m[1][0] * matrix4x4.m[3][2])
		+ (matrix4x4.m[0][0] * matrix4x4.m[1][2] * matrix4x4.m[3][1]));

	resoultMatrix.m[3][3] = 1.0f / bottom * (
		(matrix4x4.m[0][0] * matrix4x4.m[1][1] * matrix4x4.m[2][2])
		+ (matrix4x4.m[0][1] * matrix4x4.m[1][2] * matrix4x4.m[2][0])
		+ (matrix4x4.m[0][2] * matrix4x4.m[1][0] * matrix4x4.m[2][1])
		- (matrix4x4.m[0][2] * matrix4x4.m[1][1] * matrix4x4.m[2][0])
		- (matrix4x4.m[0][1] * matrix4x4.m[1][0] * matrix4x4.m[2][2])
		- (matrix4x4.m[0][0] * matrix4x4.m[1][2] * matrix4x4.m[2][1]));

	return resoultMatrix;
}

Matrix4x4 MakeViewportMatrix(float left, float top, float width, float height, float minDepth, float maxDepth)
{
	Matrix4x4 viewportMatrix4x4;

	viewportMatrix4x4.m[0][0] = width / 2.0f;
	viewportMatrix4x4.m[0][1] = 0.0f;
	viewportMatrix4x4.m[0][2] = 0.0f;
	viewportMatrix4x4.m[0][3] = 0.0f;

	viewportMatrix4x4.m[1][0] = 0.0f;
	viewportMatrix4x4.m[1][1] = -height / 2.0f;
	viewportMatrix4x4.m[1][2] = 0.0f;
	viewportMatrix4x4.m[1][3] = 0.0f;

	viewportMatrix4x4.m[2][0] = 0.0f;
	viewportMatrix4x4.m[2][1] = 0.0f;
	viewportMatrix4x4.m[2][2] = maxDepth - minDepth;
	viewportMatrix4x4.m[2][3] = 0.0f;

	viewportMatrix4x4.m[3][0] = left + width / 2.0f;
	viewportMatrix4x4.m[3][1] = top + height / 2.0f;
	viewportMatrix4x4.m[3][2] = minDepth;
	viewportMatrix4x4.m[3][3] = 1.0f;

	return viewportMatrix4x4;
}
