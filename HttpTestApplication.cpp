// HttpTestApplication.cpp : コンソール アプリケーションのエントリ ポイントを定義します。
//

#include "windows.h"
#include "Wininet.h"

#include <tchar.h>
#include <assert.h>
#include <string>
#include <string.h>
#include <sstream>
#include <locale.h>
#include <conio.h>
#include "NhConversion.h"
#include <random>

#pragma comment(lib, "Wininet.lib")

typedef std::basic_string<TCHAR>		tstring;
typedef std::basic_stringstream<TCHAR>	tstringstream;

#define URLBUFFER_SIZE		(4096)
#define	READBUFFER_SIZE		(4096)
#define FEATURE_SIZE 5
#define FRAME_SIZE 13

bool HttpRequest(tstring strUserAgent,
	tstring strUrl,
	bool bIsHttpVerbGet,
	tstring strParameter,
	tstring& rstrResult)
{
	// アウトプットの初期化
	rstrResult = tstring();

	// インプットのチェック
	if (0 == strUrl.length())
	{
		assert(!"URLが不正");
		return false;
	}

	// 変数
	HINTERNET			hInternetOpen = NULL;
	HINTERNET			hInternetConnect = NULL;
	HINTERNET			hInternetRequest = NULL;
	char*				pszOptional = NULL;
	URL_COMPONENTS		urlcomponents;
	tstring				strServer;
	tstring				strObject;
	INTERNET_PORT		nPort;
	tstring				strVerb;
	tstring				strHeaders;
	tstringstream		ssRead;

	// URL解析
	ZeroMemory(&urlcomponents, sizeof(URL_COMPONENTS));
	urlcomponents.dwStructSize = sizeof(URL_COMPONENTS);
	TCHAR szHostName[URLBUFFER_SIZE];
	TCHAR szUrlPath[URLBUFFER_SIZE];
	urlcomponents.lpszHostName = szHostName;
	urlcomponents.lpszUrlPath = szUrlPath;
	urlcomponents.dwHostNameLength = URLBUFFER_SIZE;
	urlcomponents.dwUrlPathLength = URLBUFFER_SIZE;
	if (!InternetCrackUrl(strUrl.c_str(),
		(DWORD)strUrl.length(),
		0,
		&urlcomponents))
	{	// URLの解析に失敗
		assert(!"URL解析に失敗");
		return false;
	}
	strServer = urlcomponents.lpszHostName;
	strObject = urlcomponents.lpszUrlPath;
	nPort = urlcomponents.nPort;

	// HTTPかHTTPSかそれ以外か
	DWORD dwFlags = 0;
	if (INTERNET_SCHEME_HTTP == urlcomponents.nScheme)
	{	// HTTP
		dwFlags = INTERNET_FLAG_RELOAD				// 要求されたファイル、オブジェクト、またはフォルダ一覧を、キャッシュからではなく、元のサーバーから強制的にダウンロードします。
			| INTERNET_FLAG_DONT_CACHE			// 返されたエンティティをキャシュへ追加しません。
			| INTERNET_FLAG_NO_AUTO_REDIRECT;	// HTTP だけで使用され、リダイレクトが HttpSendRequest で処理されないことを指定します。
	}
	else if (INTERNET_SCHEME_HTTPS == urlcomponents.nScheme)
	{	// HTTPS
		dwFlags = INTERNET_FLAG_RELOAD				// 要求されたファイル、オブジェクト、またはフォルダ一覧を、キャッシュからではなく、元のサーバーから強制的にダウンロードします。
			| INTERNET_FLAG_DONT_CACHE			// 返されたエンティティをキャシュへ追加しません。
			| INTERNET_FLAG_NO_AUTO_REDIRECT	// HTTP だけで使用され、リダイレクトが HttpSendRequest で処理されないことを指定します。
			| INTERNET_FLAG_SECURE						// 安全なトランザクションを使用します。これにより、SSL/PCT を使うように変換され、HTTP 要求だけで有効です。 
			| INTERNET_FLAG_IGNORE_CERT_DATE_INVALID	// INTERNET_FLAG_IGNORE_CERT_DATE_INVALID、INTERNET_FLAG_IGNORE_CERT_CN_INVALID
			| INTERNET_FLAG_IGNORE_CERT_CN_INVALID;		// は、証明書に関する警告を無視するフラグ
	}
	else
	{
		assert(!"HTTPでもHTTPSでもない");
		return false;
	}

	// GETかPOSTか
	if (bIsHttpVerbGet)
	{	// GET
		strVerb = _T("GET");
		strHeaders = _T("");
		if (0 != strParameter.length())
		{	// オブジェクトとパラメータを「?」で連結
			strObject += _T("?") + strParameter;
		}
	}
	else
	{	// POST
		strVerb = _T("POST");
		strHeaders = _T("Content-Type: application/x-www-form-urlencoded");
		if (0 != strParameter.length())
		{	// パラメータを、送信するオプションデータに変換する
			pszOptional = NhT2M(strParameter.c_str());	// char文字列に変換
		}
	}

	// WinInetの初期化
	hInternetOpen = InternetOpen(strUserAgent.c_str(),
		INTERNET_OPEN_TYPE_PRECONFIG,
		NULL, NULL, 0);
	if (NULL == hInternetOpen)
	{
		assert(!"WinInetの初期化に失敗");
		goto LABEL_ERROR;
	}

	// HTTP接続
	hInternetConnect = InternetConnect(hInternetOpen,
		strServer.c_str(),
		nPort,
		NULL,
		NULL,
		INTERNET_SERVICE_HTTP,
		0,
		0);
	if (NULL == hInternetConnect)
	{
		assert(!"HTTP接続に失敗");
		goto LABEL_ERROR;
	}

	// HTTP接続を開く
	hInternetRequest = HttpOpenRequest(hInternetConnect,
		strVerb.c_str(),
		strObject.c_str(),
		NULL,
		NULL,
		NULL,
		dwFlags,
		NULL);
	if (NULL == hInternetRequest)
	{
		assert(!"HTTP接続を開くに失敗");
		goto LABEL_ERROR;
	}

	// HTTP要求送信
	if (!HttpSendRequest(hInternetRequest,
		strHeaders.c_str(),
		(DWORD)strHeaders.length(),
		(LPVOID)((char*)pszOptional),
		pszOptional ? (DWORD)(strlen(pszOptional) * sizeof(char)) : 0))
	{
		assert(!"HTTP要求送信に失敗");
		goto LABEL_ERROR;
	}

	// HTTP要求に対応するステータスコードの取得
	DWORD dwStatusCode;
	DWORD dwLength = sizeof(DWORD);
	if (!HttpQueryInfo(hInternetRequest,
		HTTP_QUERY_STATUS_CODE | HTTP_QUERY_FLAG_NUMBER,
		&dwStatusCode,
		&dwLength,
		0))
	{
		assert(!"HTTP要求に対応するステータスコードの取得に失敗");
		goto LABEL_ERROR;
	}
	if (HTTP_STATUS_OK != dwStatusCode)
	{
		assert(!"ステータスコードがOKでない");
		goto LABEL_ERROR;
	}

	// HTTPファイル読み込み
	char szReadBuffer[READBUFFER_SIZE + 1];
	while (1)
	{
		DWORD dwRead = 0;
		if (!InternetReadFile(hInternetRequest, szReadBuffer, READBUFFER_SIZE, &dwRead))
		{
			assert(!"HTTPファイル読み込みに失敗");
			goto LABEL_ERROR;
		}
		if (0 == dwRead)
		{
			break;
		}
		szReadBuffer[dwRead] = '\0';	// 終端文字「\0」の付加
		size_t length = dwRead + 1;
		LPWSTR	pszWideChar = (LPWSTR)malloc(length * sizeof(WCHAR));
		MultiByteToWideChar(CP_UTF8,	// CODE PAGE: UTF-8
			0,
			szReadBuffer,
			-1,
			pszWideChar,
			(int)length);	// UTF-8文字列をANSI文字列に変換
		TCHAR* pszTchar = NhW2T(pszWideChar);	// WideChar文字列をTCHAR文字列に変換
		ssRead << pszTchar;	// ストリーム文字列に流し込む
		free(pszTchar);
		free(pszWideChar);
	}

	// ストリーム文字列を、出力文字列に変換
	rstrResult = ssRead.str().c_str();

	if (pszOptional){ free(pszOptional); }
	InternetCloseHandle(hInternetRequest);
	InternetCloseHandle(hInternetConnect);
	InternetCloseHandle(hInternetOpen);
	return true;

LABEL_ERROR:
	if (pszOptional){ free(pszOptional); }
	InternetCloseHandle(hInternetRequest);
	InternetCloseHandle(hInternetConnect);
	InternetCloseHandle(hInternetOpen);
	return false;
}
/*
enum PARAM{
	PROC_ID = 1,
	DEVISE_ID = 2,
    FEATURE_ID = 3,
	FRAME = 4,
	VALUE = 5
};

enum FEATURE{
	HIP = 1,
	LEFTKNEE = 2, 
	LEFTHEEL = 3,
	RIGHTKNEE = 4,
	RIGHTHEEL = 5
};

TCHAR* setProcIdStr(int value){
	TCHAR* str = new TCHAR[50];
	_stprintf_s(str, 50, _T("proc_id=%d"), value);
	return str;
}

tstring setDeviseIdStr(int value){
	TCHAR* str = new TCHAR[50];
	_stprintf_s(str, 50, _T("devise_id=%d"), value);
	return str;
}

tstring setFeatureIdStr(int value){
	TCHAR* str = new TCHAR[50];
	_stprintf_s(str, 50, _T("feature_id=%d"), value);
	return str;
}

tstring setFrameStr(int value){
	TCHAR* str = new TCHAR[50];
	_stprintf_s(str, 50, _T("frame=%d"), value);
	return str;
}

tstring setValueStr(float value){
	TCHAR* str = new TCHAR[50];
	_stprintf_s(str, 50, _T("value=%f"), value);
	return str;
}*/
/*
int _tmain(int argc, _TCHAR* argv[])
{
	tstring strUserAgent = _T("HttpRequestTest");
	tstring strUrl = _T("https://kinect-walking-api.herokuapp.com/index");
	bool bIsHttpVerbGet = true;

	const int proc_id = 104;
	const int devise_id = 1;
	//const int feature_size = 5;
	const int features[FEATURE_SIZE] = { HIP, LEFTKNEE, LEFTHEEL, RIGHTKNEE, RIGHTHEEL };
	//const int nFrame = 13;
	float values[FEATURE_SIZE][FRAME_SIZE] = {};
	for (int i = 0; i < FEATURE_SIZE; i++){
		for (int j = 0; j < FRAME_SIZE; j++){
			values[i][j] = rand();
		}
	}
	tstring anp = _T("&");
	tstring procIdParams = setProcIdStr(proc_id);
	tstring deviseIdParams = setDeviseIdStr(devise_id);
	tstring strResult;
	for (int i = 0; i < FEATURE_SIZE; i++){
		for (int j = 0; j < FRAME_SIZE; j++){
			tstring featureIdParams = setFeatureIdStr(i+1);
			tstring frameParams = setFrameStr(j+1);
			tstring valueParams = setValueStr(values[i][j]);
			tstring strParameter = procIdParams + anp + deviseIdParams + anp + featureIdParams + anp + frameParams + anp + valueParams;
			if (!HttpRequest(strUserAgent, strUrl, bIsHttpVerbGet,strParameter, strResult))
			{
				return false;
			}
		}
	}

	setlocale(LC_ALL, "Japanese");
	_tprintf(_T("%s"), strResult.c_str());

	_getch();

	return 0;
}*/